#include "CameraStreamServer.h"

#include "AirBlueprintLib.h"
#include "UnrealImageCapture.h"

#include "Async/TaskGraphInterfaces.h"
#include "Common/TcpListener.h"
#include "HAL/PlatformProcess.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Misc/ScopeExit.h"
#include "Modules/ModuleManager.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <map>

namespace
{
    using ImageRequest = msr::airlib::ImageCaptureBase::ImageRequest;
    using ImageResponse = msr::airlib::ImageCaptureBase::ImageResponse;
    using ImageType = msr::airlib::ImageCaptureBase::ImageType;

    bool IsFloatImageType(int32 image_type)
    {
        return image_type == static_cast<int32>(ImageType::DepthPlanar) ||
               image_type == static_cast<int32>(ImageType::DepthPerspective) ||
               image_type == static_cast<int32>(ImageType::DisparityNormalized);
    }

    bool IsDepthImageType(int32 image_type)
    {
        return image_type == static_cast<int32>(ImageType::DepthPlanar) ||
               image_type == static_cast<int32>(ImageType::DepthPerspective);
    }

    FString EscapeJson(const FString& value)
    {
        FString result;
        result.Reserve(value.Len());
        for (const TCHAR character : value) {
            switch (character) {
            case TEXT('\\'): result += TEXT("\\\\"); break;
            case TEXT('"'): result += TEXT("\\\""); break;
            case TEXT('\b'): result += TEXT("\\b"); break;
            case TEXT('\f'): result += TEXT("\\f"); break;
            case TEXT('\n'): result += TEXT("\\n"); break;
            case TEXT('\r'): result += TEXT("\\r"); break;
            case TEXT('\t'): result += TEXT("\\t"); break;
            default:
                if (character >= 0 && character < 0x20)
                    result += FString::Printf(TEXT("\\u%04X"), static_cast<uint32>(character));
                else
                    result.AppendChar(character);
                break;
            }
        }
        return result;
    }

    FString HttpStatusBody(const FString& message)
    {
        return FString::Printf(TEXT("{\"error\":\"%s\"}"), *EscapeJson(message));
    }
}

struct FCameraStreamServer::FFrame
{
    std::shared_ptr<const std::vector<uint8>> Jpeg;
    std::shared_ptr<const std::vector<uint8>> Raw;
    uint64 Sequence = 0;
    uint64 AirSimTimestamp = 0;
    int32 Width = 0;
    int32 Height = 0;
    int32 Channels = 0;
    FString Dtype;
    FString ColorOrder;
    double Fps = 0.0;
    double LatencyMs = 0.0;
};

struct FCameraStreamServer::FSource
{
    FString VehicleName;
    FString CameraName;
    FString AnnotationName;
    FString ImageTypeLabel;
    FString StreamPath;
    FString RawPath;
    FString SnapshotPath;
    int32 ImageType = 0;
    const UnrealImageCapture* ImageCapture = nullptr;

    std::atomic<int32> Subscribers{ 0 };
    mutable std::mutex FrameMutex;
    std::condition_variable FrameCondition;
    std::shared_ptr<const FFrame> LatestFrame;
    FString LastError;
    std::deque<double> CaptureTimes;
    FCaptureWorker* Worker = nullptr;
};

struct FCameraStreamServer::FClientThread
{
    std::atomic<bool> Finished{ false };
    std::thread Thread;
};

class FCameraStreamServer::FCaptureWorker
{
public:
    FCaptureWorker(
        const UnrealImageCapture* image_capture,
        std::vector<std::shared_ptr<FSource>> sources,
        const msr::airlib::AirSimSettings::CameraHostSetting& settings,
        IImageWrapperModule* image_wrapper_module)
        : ImageCapture_(image_capture),
          Sources_(std::move(sources)),
          Settings_(settings),
          ImageWrapperModule_(image_wrapper_module),
          Cancellation_(std::make_shared<std::atomic<bool>>(false))
    {
        for (const std::shared_ptr<FSource>& source : Sources_)
            source->Worker = this;
    }

    ~FCaptureWorker()
    {
        Stop();
    }

    void Start()
    {
        if (Running_.exchange(true))
            return;
        Cancellation_->store(false);
        Finished_ = false;
        Thread_ = std::thread(&FCaptureWorker::Run, this);
    }

    void Stop()
    {
        if (!Running_.exchange(false))
            return;
        Cancellation_->store(true);
        WakeCondition_.notify_all();
        for (const std::shared_ptr<FSource>& source : Sources_)
            source->FrameCondition.notify_all();
        if (Thread_.joinable()) {
            // EndPlay runs on the game thread. The capture worker may be
            // waiting for a queued game-thread cancellation task, so drain
            // those tasks until it releases all camera/render references.
            if (IsInGameThread()) {
                while (!Finished_.load()) {
                    FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
                    FPlatformProcess::SleepNoStats(0.001f);
                }
            }
            Thread_.join();
        }
    }

    void Wake()
    {
        WakeCondition_.notify_all();
    }

private:
    bool HasSubscribers() const
    {
        for (const std::shared_ptr<FSource>& source : Sources_) {
            if (source->Subscribers.load() > 0)
                return true;
        }
        return false;
    }

    bool EncodeJpeg(const std::vector<uint8>& rgb, int32 width, int32 height, std::vector<uint8>& output) const
    {
        const int64 pixel_count = static_cast<int64>(width) * static_cast<int64>(height);
        if (ImageWrapperModule_ == nullptr || width <= 0 || height <= 0 ||
            rgb.size() != static_cast<size_t>(pixel_count) * 3)
            return false;

        // UE 5.5's JPEG wrapper accepts RGBA/BGRA input, so expand the
        // renderer's packed RGB output only for the preview encoder. The raw
        // endpoint remains the original exact RGB byte buffer.
        std::vector<uint8> rgba(static_cast<size_t>(pixel_count) * 4);
        for (int64 pixel = 0; pixel < pixel_count; ++pixel) {
            const size_t rgb_offset = static_cast<size_t>(pixel) * 3;
            const size_t rgba_offset = static_cast<size_t>(pixel) * 4;
            rgba[rgba_offset] = rgb[rgb_offset];
            rgba[rgba_offset + 1] = rgb[rgb_offset + 1];
            rgba[rgba_offset + 2] = rgb[rgb_offset + 2];
            rgba[rgba_offset + 3] = 255;
        }

        const TSharedPtr<IImageWrapper> wrapper = ImageWrapperModule_->CreateImageWrapper(EImageFormat::JPEG, TEXT("AirSimCameraHost"));
        if (!wrapper.IsValid() ||
            !wrapper->SetRaw(rgba.data(), static_cast<int64>(rgba.size()), width, height, ERGBFormat::RGBA, 8)) {
            return false;
        }

        const TArray64<uint8> compressed = wrapper->GetCompressed(Settings_.jpeg_quality);
        if (compressed.Num() <= 0)
            return false;
        output.assign(compressed.GetData(), compressed.GetData() + compressed.Num());
        return true;
    }

    std::vector<uint8> MakeFloatPreview(const ImageResponse& response, int32 image_type) const
    {
        const int64 pixel_count = static_cast<int64>(response.width) * static_cast<int64>(response.height);
        std::vector<uint8> preview(static_cast<size_t>(pixel_count) * 3, 0);
        if (pixel_count <= 0 || response.image_data_float.size() != static_cast<size_t>(pixel_count))
            return preview;

        const bool invert = IsDepthImageType(image_type);
        const float scale = image_type == static_cast<int32>(ImageType::DisparityNormalized)
            ? 1.0f
            : FMath::Max(Settings_.float_preview_max, 0.000001f);
        for (int64 index = 0; index < pixel_count; ++index) {
            const float value = response.image_data_float[static_cast<size_t>(index)];
            float normalized = FMath::IsFinite(value) ? FMath::Clamp(value / scale, 0.0f, 1.0f) : 0.0f;
            if (invert && FMath::IsFinite(value))
                normalized = 1.0f - normalized;
            const uint8 level = static_cast<uint8>(FMath::RoundToInt(normalized * 255.0f));
            const size_t offset = static_cast<size_t>(index) * 3;
            preview[offset] = level;
            preview[offset + 1] = level;
            preview[offset + 2] = level;
        }
        return preview;
    }

    void SetError(const std::shared_ptr<FSource>& source, const FString& error)
    {
        std::lock_guard<std::mutex> lock(source->FrameMutex);
        source->LastError = error;
        source->FrameCondition.notify_all();
    }

    void Publish(const std::shared_ptr<FSource>& source, ImageResponse& response, double latency_ms)
    {
        if (response.width <= 0 || response.height <= 0) {
            SetError(source, UTF8_TO_TCHAR(response.message.c_str()));
            return;
        }

        const bool is_float = IsFloatImageType(source->ImageType);
        std::vector<uint8> preview;
        std::vector<uint8> raw;
        FString dtype;
        FString color_order;
        int32 channels = 0;

        if (is_float) {
            const size_t float_count = response.image_data_float.size();
            raw.resize(float_count * sizeof(float));
            if (!raw.empty())
                std::memcpy(raw.data(), response.image_data_float.data(), raw.size());
            preview = MakeFloatPreview(response, source->ImageType);
            dtype = TEXT("float32");
            color_order = TEXT("FLOAT32");
            channels = 1;
        }
        else {
            raw = std::move(response.image_data_uint8);
            preview = raw;
            dtype = TEXT("uint8");
            color_order = TEXT("RGB");
            channels = 3;
        }

        const size_t expected_rgb_size = static_cast<size_t>(response.width) * static_cast<size_t>(response.height) * 3;
        if (preview.size() != expected_rgb_size) {
            SetError(source, FString::Printf(TEXT("Unexpected preview byte count %llu (expected %llu)"),
                                             static_cast<uint64>(preview.size()), static_cast<uint64>(expected_rgb_size)));
            return;
        }

        std::vector<uint8> jpeg;
        if (!EncodeJpeg(preview, response.width, response.height, jpeg)) {
            SetError(source, TEXT("JPEG encoding failed"));
            return;
        }

        const double now = FPlatformTime::Seconds();
        std::shared_ptr<FFrame> frame = std::make_shared<FFrame>();
        frame->Jpeg = std::make_shared<const std::vector<uint8>>(std::move(jpeg));
        frame->Raw = std::make_shared<const std::vector<uint8>>(std::move(raw));
        frame->AirSimTimestamp = static_cast<uint64>(response.time_stamp);
        frame->Width = response.width;
        frame->Height = response.height;
        frame->Channels = channels;
        frame->Dtype = std::move(dtype);
        frame->ColorOrder = std::move(color_order);
        frame->LatencyMs = latency_ms;

        {
            std::lock_guard<std::mutex> lock(source->FrameMutex);
            source->CaptureTimes.push_back(now);
            while (!source->CaptureTimes.empty() && now - source->CaptureTimes.front() > 2.0)
                source->CaptureTimes.pop_front();
            if (source->CaptureTimes.size() >= 2) {
                const double elapsed = source->CaptureTimes.back() - source->CaptureTimes.front();
                frame->Fps = elapsed > 0.0 ? static_cast<double>(source->CaptureTimes.size() - 1) / elapsed : 0.0;
            }
            frame->Sequence = source->LatestFrame ? source->LatestFrame->Sequence + 1 : 1;
            source->LatestFrame = frame;
            source->LastError.Empty();
        }
        source->FrameCondition.notify_all();
    }

    void Run()
    {
        ON_SCOPE_EXIT
        {
            Finished_ = true;
        };

        while (Running_.load()) {
            if (!HasSubscribers()) {
                std::unique_lock<std::mutex> lock(WakeMutex_);
                WakeCondition_.wait_for(lock, std::chrono::milliseconds(250), [this]() {
                    return !Running_.load() || HasSubscribers();
                });
                continue;
            }

            const double started = FPlatformTime::Seconds();
            std::vector<std::shared_ptr<FSource>> active_sources;
            std::vector<ImageRequest> requests;
            for (const std::shared_ptr<FSource>& source : Sources_) {
                if (source->Subscribers.load() <= 0)
                    continue;
                active_sources.push_back(source);
                requests.emplace_back(
                    TCHAR_TO_UTF8(*source->CameraName),
                    static_cast<ImageType>(source->ImageType),
                    IsFloatImageType(source->ImageType),
                    false,
                    TCHAR_TO_UTF8(*source->AnnotationName),
                    false);
            }

            if (requests.empty())
                continue;

            try {
                std::vector<ImageResponse> responses;
                ImageCapture_->getImages(requests, responses, Cancellation_);
                if (!Running_.load())
                    break;

                const double latency_ms = (FPlatformTime::Seconds() - started) * 1000.0;
                if (responses.size() != active_sources.size()) {
                    for (const std::shared_ptr<FSource>& source : active_sources)
                        SetError(source, TEXT("AirSim returned an unexpected number of camera responses"));
                }
                else {
                    for (size_t index = 0; index < responses.size(); ++index)
                        Publish(active_sources[index], responses[index], latency_ms);
                }
            }
            catch (const std::exception& exception) {
                const FString error = UTF8_TO_TCHAR(exception.what());
                for (const std::shared_ptr<FSource>& source : active_sources)
                    SetError(source, error);
            }
            catch (...) {
                for (const std::shared_ptr<FSource>& source : active_sources)
                    SetError(source, TEXT("Unknown native camera capture error"));
            }

            const double frame_period = 1.0 / FMath::Max(0.1f, Settings_.target_fps);
            const double remaining = frame_period - (FPlatformTime::Seconds() - started);
            if (remaining > 0.0) {
                std::unique_lock<std::mutex> lock(WakeMutex_);
                WakeCondition_.wait_for(lock, std::chrono::duration<double>(remaining), [this]() {
                    return !Running_.load();
                });
            }
        }
    }

private:
    const UnrealImageCapture* ImageCapture_ = nullptr;
    std::vector<std::shared_ptr<FSource>> Sources_;
    msr::airlib::AirSimSettings::CameraHostSetting Settings_;
    IImageWrapperModule* ImageWrapperModule_ = nullptr;
    std::atomic<bool> Running_{ false };
    std::atomic<bool> Finished_{ true };
    std::thread Thread_;
    std::mutex WakeMutex_;
    std::condition_variable WakeCondition_;
    std::shared_ptr<std::atomic<bool>> Cancellation_;
};

FCameraStreamServer::FCameraStreamServer(
    const msr::airlib::AirSimSettings::CameraHostSetting& settings,
    std::vector<FAirSimHostedCamera> hosted_cameras)
    : Settings_(settings)
{
    std::set<FString> used_paths;
    for (const FAirSimHostedCamera& camera : hosted_cameras) {
        if (camera.ImageCapture == nullptr)
            continue;

        std::shared_ptr<FSource> source = std::make_shared<FSource>();
        source->VehicleName = camera.VehicleName;
        source->CameraName = camera.CameraName;
        source->AnnotationName = camera.AnnotationName;
        source->ImageType = camera.ImageType;
        source->ImageCapture = camera.ImageCapture;
        source->ImageTypeLabel = ImageTypeName(camera.ImageType);

        FString type_segment = source->ImageTypeLabel.ToLower();
        if (!source->AnnotationName.IsEmpty())
            type_segment += TEXT("-") + UrlEncodeSegment(source->AnnotationName);
        const FString vehicle_segment = source->VehicleName.IsEmpty() ? TEXT("default") : UrlEncodeSegment(source->VehicleName);
        source->StreamPath = FString::Printf(
            TEXT("/camera/%s/%s/%s"),
            *vehicle_segment,
            *UrlEncodeSegment(source->CameraName),
            *type_segment);

        if (used_paths.find(source->StreamPath) != used_paths.end()) {
            UE_LOG(LogTemp, Error, TEXT("AirSim camera host duplicate route ignored: %s"), *source->StreamPath);
            continue;
        }
        used_paths.insert(source->StreamPath);
        source->RawPath = source->StreamPath + TEXT("/raw");
        source->SnapshotPath = source->StreamPath + TEXT("/snapshot.jpg");
        Sources_.push_back(std::move(source));
    }
}

FCameraStreamServer::~FCameraStreamServer()
{
    Stop();
}

bool FCameraStreamServer::Start()
{
    if (Sources_.empty())
        return false;
    if (Running_.exchange(true))
        return true;

    FIPv4Address bind_address;
    if (!FIPv4Address::Parse(UTF8_TO_TCHAR(Settings_.bind_address.c_str()), bind_address)) {
        Running_ = false;
        UE_LOG(LogTemp, Error, TEXT("AirSim camera host has invalid BindAddress: %s"), UTF8_TO_TCHAR(Settings_.bind_address.c_str()));
        return false;
    }

    ImageWrapperModule_ = &FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));

    std::map<const UnrealImageCapture*, std::vector<std::shared_ptr<FSource>>> grouped_sources;
    for (const std::shared_ptr<FSource>& source : Sources_)
        grouped_sources[source->ImageCapture].push_back(source);
    for (auto& pair : grouped_sources) {
        CaptureWorkers_.push_back(std::make_unique<FCaptureWorker>(
            pair.first, std::move(pair.second), Settings_, ImageWrapperModule_));
    }
    for (const std::unique_ptr<FCaptureWorker>& worker : CaptureWorkers_)
        worker->Start();

    Listener_ = MakeUnique<FTcpListener>(
        FIPv4Endpoint(bind_address, static_cast<uint16>(Settings_.port)),
        FTimespan::FromMilliseconds(100),
        true);
    Listener_->OnConnectionAccepted().BindRaw(this, &FCameraStreamServer::HandleAcceptedConnection);

    for (int32 attempt = 0; attempt < 100 && !Listener_->IsActive(); ++attempt)
        FPlatformProcess::Sleep(0.01f);
    if (!Listener_->IsActive()) {
        UE_LOG(LogTemp, Error, TEXT("AirSim camera host could not listen on %s:%d"),
               UTF8_TO_TCHAR(Settings_.bind_address.c_str()), Settings_.port);
        Stop();
        return false;
    }

    UE_LOG(LogTemp, Display, TEXT("AirSim camera host listening on http://%s:%d with %d hosted camera streams"),
           UTF8_TO_TCHAR(Settings_.bind_address.c_str()), Settings_.port, static_cast<int32>(Sources_.size()));
    for (const std::shared_ptr<FSource>& source : Sources_)
        UE_LOG(LogTemp, Display, TEXT("AirSim camera stream: %s"), *source->StreamPath);
    return true;
}

void FCameraStreamServer::Stop()
{
    if (!Running_.exchange(false))
        return;

    if (Listener_.IsValid()) {
        Listener_->OnConnectionAccepted().Unbind();
        Listener_->Stop();
        Listener_.Reset();
    }

    for (const std::unique_ptr<FCaptureWorker>& worker : CaptureWorkers_)
        worker->Stop();
    for (const std::shared_ptr<FSource>& source : Sources_)
        source->FrameCondition.notify_all();

    std::vector<std::shared_ptr<FClientThread>> client_threads;
    {
        std::lock_guard<std::mutex> lock(ClientsMutex_);
        for (FSocket* socket : ClientSockets_) {
            if (socket != nullptr) {
                socket->Shutdown(ESocketShutdownMode::ReadWrite);
                socket->Close();
            }
        }
        client_threads.swap(ClientThreads_);
    }
    for (const std::shared_ptr<FClientThread>& client : client_threads) {
        if (client && client->Thread.joinable())
            client->Thread.join();
    }
    CaptureWorkers_.clear();
    ImageWrapperModule_ = nullptr;
}

bool FCameraStreamServer::IsRunning() const
{
    return Running_.load();
}

bool FCameraStreamServer::HandleAcceptedConnection(FSocket* socket, const FIPv4Endpoint& remote_endpoint)
{
    (void)remote_endpoint;
    if (!Running_.load() || socket == nullptr)
        return false;

    socket->SetNonBlocking(true);
    socket->SetNoDelay(true);
    int32 actual_send_buffer_size = 0;
    socket->SetSendBufferSize(2 * 1024 * 1024, actual_send_buffer_size);

    const std::shared_ptr<FClientThread> client = std::make_shared<FClientThread>();
    std::lock_guard<std::mutex> lock(ClientsMutex_);
    ReapClientThreadsLocked();
    // Stop() can race a listener callback that already passed the first
    // Running_ check. Re-check while holding the same mutex used to collect
    // client threads so no joinable thread can be added after shutdown swaps
    // the collection.
    if (!Running_.load())
        return false;
    if (ClientSockets_.size() >= static_cast<size_t>(Settings_.max_connections)) {
        UE_LOG(LogTemp, Warning, TEXT("AirSim camera host rejected a connection because MaxConnections=%d was reached"),
               Settings_.max_connections);
        return false;
    }

    ClientSockets_.insert(socket);
    try {
        client->Thread = std::thread([this, socket, client]() {
            try {
                HandleConnection(socket);
            }
            catch (const std::exception& exception) {
                UE_LOG(LogTemp, Error, TEXT("AirSim camera host client failed: %s"), UTF8_TO_TCHAR(exception.what()));
                FinishConnection(socket);
            }
            catch (...) {
                UE_LOG(LogTemp, Error, TEXT("AirSim camera host client failed with an unknown error"));
                FinishConnection(socket);
            }
            client->Finished = true;
        });
    }
    catch (const std::exception& exception) {
        ClientSockets_.erase(socket);
        UE_LOG(LogTemp, Error, TEXT("AirSim camera host could not create a client thread: %s"),
               UTF8_TO_TCHAR(exception.what()));
        return false;
    }
    ClientThreads_.push_back(client);
    return true;
}

void FCameraStreamServer::ReapClientThreadsLocked()
{
    for (auto iterator = ClientThreads_.begin(); iterator != ClientThreads_.end();) {
        const std::shared_ptr<FClientThread>& client = *iterator;
        if (client && client->Finished.load()) {
            if (client->Thread.joinable())
                client->Thread.join();
            iterator = ClientThreads_.erase(iterator);
        }
        else {
            ++iterator;
        }
    }
}

void FCameraStreamServer::FinishConnection(FSocket* socket)
{
    {
        std::lock_guard<std::mutex> lock(ClientsMutex_);
        ClientSockets_.erase(socket);
    }
    if (socket != nullptr) {
        socket->Shutdown(ESocketShutdownMode::ReadWrite);
        socket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(socket);
    }
}

void FCameraStreamServer::HandleConnection(FSocket* socket)
{
    FString target;
    if (!ReadRequest(socket, target)) {
        FinishConnection(socket);
        return;
    }

    FString path;
    bool has_after_sequence = false;
    bool query_is_valid = true;
    const uint64 after_sequence = ParseAfterSequence(target, path, has_after_sequence, query_is_valid);
    if (!query_is_valid) {
        SendTextResponse(socket, 400, TEXT("Bad Request"), TEXT("application/json; charset=utf-8"),
                         HttpStatusBody(TEXT("after must be an unsigned 64-bit integer")));
        FinishConnection(socket);
        return;
    }
    if (path == TEXT("/"))
        SendTextResponse(socket, 200, TEXT("OK"), TEXT("text/html; charset=utf-8"), BuildDashboardHtml());
    else if (path == TEXT("/api/cameras"))
        SendTextResponse(socket, 200, TEXT("OK"), TEXT("application/json; charset=utf-8"), BuildInventoryJson());
    else if (path == TEXT("/api/status"))
        SendTextResponse(socket, 200, TEXT("OK"), TEXT("application/json; charset=utf-8"), BuildStatusJson());
    else {
        const std::shared_ptr<FSource> source = FindSource(path);
        if (source && path == source->StreamPath)
            ServeMjpeg(socket, source);
        else if (source && path == source->SnapshotPath)
            ServeJpeg(socket, source, after_sequence, !has_after_sequence);
        else if (source && path == source->RawPath)
            ServeRaw(socket, source, after_sequence, !has_after_sequence);
        else
            SendTextResponse(socket, 404, TEXT("Not Found"), TEXT("application/json; charset=utf-8"), HttpStatusBody(TEXT("route not found")));
    }

    FinishConnection(socket);
}

bool FCameraStreamServer::ReadRequest(FSocket* socket, FString& out_target) const
{
    std::vector<uint8> request;
    request.reserve(4096);
    bool headers_complete = false;
    const double deadline = FPlatformTime::Seconds() + 5.0;
    while (Running_.load() && FPlatformTime::Seconds() < deadline && request.size() < 16384) {
        uint32 pending = 0;
        if (!socket->HasPendingData(pending) || pending == 0) {
            FPlatformProcess::Sleep(0.005f);
            continue;
        }
        uint8 buffer[2048];
        int32 bytes_read = 0;
        if (!socket->Recv(buffer, FMath::Min<int32>(static_cast<int32>(pending), UE_ARRAY_COUNT(buffer)), bytes_read) || bytes_read <= 0)
            return false;
        request.insert(request.end(), buffer, buffer + bytes_read);
        if (request.size() >= 4) {
            const std::string text(reinterpret_cast<const char*>(request.data()), request.size());
            if (text.find("\r\n\r\n") != std::string::npos) {
                headers_complete = true;
                break;
            }
        }
    }
    if (!headers_complete)
        return false;

    const std::string text(reinterpret_cast<const char*>(request.data()), request.size());
    const size_t line_end = text.find("\r\n");
    const std::string request_line = text.substr(0, line_end);
    const size_t first_space = request_line.find(' ');
    const size_t second_space = first_space == std::string::npos ? std::string::npos : request_line.find(' ', first_space + 1);
    if (first_space == std::string::npos || second_space == std::string::npos || request_line.substr(0, first_space) != "GET")
        return false;
    out_target = UTF8_TO_TCHAR(request_line.substr(first_space + 1, second_space - first_space - 1).c_str());
    return true;
}

bool FCameraStreamServer::SendAll(FSocket* socket, const uint8* data, int64 size) const
{
    int64 offset = 0;
    while (Running_.load() && offset < size) {
        if (!socket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromSeconds(2)))
            return false;
        int32 sent = 0;
        const int32 chunk = static_cast<int32>(FMath::Min<int64>(size - offset, 1024 * 1024));
        if (!socket->Send(data + offset, chunk, sent) || sent <= 0)
            return false;
        offset += sent;
    }
    return offset == size;
}

bool FCameraStreamServer::SendTextResponse(
    FSocket* socket, int32 status_code, const TCHAR* status_text, const FString& content_type, const FString& body) const
{
    FTCHARToUTF8 body_utf8(*body);
    const FString header = FString::Printf(
        TEXT("HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"),
        status_code, status_text, *content_type, body_utf8.Length());
    FTCHARToUTF8 header_utf8(*header);
    return SendAll(socket, reinterpret_cast<const uint8*>(header_utf8.Get()), header_utf8.Length()) &&
           SendAll(socket, reinterpret_cast<const uint8*>(body_utf8.Get()), body_utf8.Length());
}

bool FCameraStreamServer::SendBinaryResponse(
    FSocket* socket,
    int32 status_code,
    const TCHAR* status_text,
    const FString& content_type,
    const std::shared_ptr<const std::vector<uint8>>& body,
    const FString& extra_headers) const
{
    const int64 body_size = body ? static_cast<int64>(body->size()) : 0;
    const FString header = FString::Printf(
        TEXT("HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %lld\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Expose-Headers: X-AirSim-Sequence, X-AirSim-Timestamp, X-AirSim-Capture-FPS, X-AirSim-Latency-Ms, X-AirSim-Width, X-AirSim-Height, X-AirSim-Channels, X-AirSim-Dtype, X-AirSim-Color-Order\r\nCache-Control: no-store\r\n%sConnection: close\r\n\r\n"),
        status_code, status_text, *content_type, body_size, *extra_headers);
    FTCHARToUTF8 header_utf8(*header);
    if (!SendAll(socket, reinterpret_cast<const uint8*>(header_utf8.Get()), header_utf8.Length()))
        return false;
    return body_size == 0 || SendAll(socket, body->data(), body_size);
}

std::shared_ptr<const FCameraStreamServer::FFrame> FCameraStreamServer::WaitForFrame(
    const std::shared_ptr<FSource>& source, uint64 after_sequence, double timeout_seconds) const
{
    const auto timeout = std::chrono::duration<double>(timeout_seconds);
    std::unique_lock<std::mutex> lock(source->FrameMutex);
    source->FrameCondition.wait_for(lock, timeout, [this, &source, after_sequence]() {
        return !Running_.load() || (source->LatestFrame && source->LatestFrame->Sequence > after_sequence);
    });
    if (source->LatestFrame && source->LatestFrame->Sequence > after_sequence)
        return source->LatestFrame;
    return nullptr;
}

uint64 FCameraStreamServer::BeginSubscription(
    const std::shared_ptr<FSource>& source, uint64 after_sequence, bool require_fresh_frame)
{
    std::lock_guard<std::mutex> lock(source->FrameMutex);
    if (require_fresh_frame && source->LatestFrame)
        after_sequence = FMath::Max(after_sequence, source->LatestFrame->Sequence);

    // Clear samples before publishing can resume so the displayed FPS never
    // includes time during which this source had no consumers.
    if (source->Subscribers.fetch_add(1) == 0)
        source->CaptureTimes.clear();
    return after_sequence;
}

void FCameraStreamServer::EndSubscription(const std::shared_ptr<FSource>& source)
{
    const int32 previous = source->Subscribers.fetch_sub(1);
    if (previous <= 0) {
        source->Subscribers = 0;
        UE_LOG(LogTemp, Error, TEXT("AirSim camera host detected an unbalanced subscription on %s"), *source->StreamPath);
    }
}

void FCameraStreamServer::ServeMjpeg(FSocket* socket, const std::shared_ptr<FSource>& source)
{
    uint64 sequence = BeginSubscription(source, 0, true);
    ON_SCOPE_EXIT
    {
        EndSubscription(source);
    };
    source->Worker->Wake();

    const FString header = TEXT("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=airsim-frame\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
    FTCHARToUTF8 header_utf8(*header);
    bool connected = SendAll(socket, reinterpret_cast<const uint8*>(header_utf8.Get()), header_utf8.Length());
    while (connected && Running_.load()) {
        const std::shared_ptr<const FFrame> frame = WaitForFrame(source, sequence, 10.0);
        if (!frame)
            break;
        sequence = frame->Sequence;
        const FString part_header = FString::Printf(
            TEXT("--airsim-frame\r\nContent-Type: image/jpeg\r\nContent-Length: %llu\r\nX-AirSim-Sequence: %llu\r\nX-AirSim-Timestamp: %llu\r\nX-AirSim-Capture-FPS: %.3f\r\n\r\n"),
            static_cast<uint64>(frame->Jpeg->size()), frame->Sequence, frame->AirSimTimestamp, frame->Fps);
        FTCHARToUTF8 part_utf8(*part_header);
        connected = SendAll(socket, reinterpret_cast<const uint8*>(part_utf8.Get()), part_utf8.Length()) &&
                    SendAll(socket, frame->Jpeg->data(), static_cast<int64>(frame->Jpeg->size())) &&
                    SendAll(socket, reinterpret_cast<const uint8*>("\r\n"), 2);
    }
}

void FCameraStreamServer::ServeJpeg(
    FSocket* socket,
    const std::shared_ptr<FSource>& source,
    uint64 after_sequence,
    bool require_fresh_frame)
{
    after_sequence = BeginSubscription(source, after_sequence, require_fresh_frame);
    ON_SCOPE_EXIT
    {
        EndSubscription(source);
    };
    source->Worker->Wake();
    const std::shared_ptr<const FFrame> frame = WaitForFrame(source, after_sequence, 10.0);
    if (!frame) {
        FString error;
        {
            std::lock_guard<std::mutex> lock(source->FrameMutex);
            error = source->LastError.IsEmpty() ? TEXT("Timed out waiting for camera frame") : source->LastError;
        }
        SendTextResponse(socket, 503, TEXT("Service Unavailable"), TEXT("application/json; charset=utf-8"), HttpStatusBody(error));
    }
    else {
        const FString headers = FString::Printf(
            TEXT("X-AirSim-Sequence: %llu\r\nX-AirSim-Timestamp: %llu\r\nX-AirSim-Capture-FPS: %.3f\r\nX-AirSim-Latency-Ms: %.2f\r\n"),
            frame->Sequence, frame->AirSimTimestamp, frame->Fps, frame->LatencyMs);
        SendBinaryResponse(socket, 200, TEXT("OK"), TEXT("image/jpeg"), frame->Jpeg, headers);
    }
}

void FCameraStreamServer::ServeRaw(
    FSocket* socket,
    const std::shared_ptr<FSource>& source,
    uint64 after_sequence,
    bool require_fresh_frame)
{
    after_sequence = BeginSubscription(source, after_sequence, require_fresh_frame);
    ON_SCOPE_EXIT
    {
        EndSubscription(source);
    };
    source->Worker->Wake();
    const std::shared_ptr<const FFrame> frame = WaitForFrame(source, after_sequence, 10.0);
    if (!frame) {
        FString error;
        {
            std::lock_guard<std::mutex> lock(source->FrameMutex);
            error = source->LastError.IsEmpty() ? TEXT("Timed out waiting for camera frame") : source->LastError;
        }
        SendTextResponse(socket, 503, TEXT("Service Unavailable"), TEXT("application/json; charset=utf-8"), HttpStatusBody(error));
    }
    else {
        const FString headers = FString::Printf(
            TEXT("X-AirSim-Sequence: %llu\r\nX-AirSim-Timestamp: %llu\r\nX-AirSim-Capture-FPS: %.3f\r\nX-AirSim-Latency-Ms: %.2f\r\nX-AirSim-Width: %d\r\nX-AirSim-Height: %d\r\nX-AirSim-Channels: %d\r\nX-AirSim-Dtype: %s\r\nX-AirSim-Color-Order: %s\r\nX-AirSim-Endianness: little\r\n"),
            frame->Sequence, frame->AirSimTimestamp, frame->Fps, frame->LatencyMs,
            frame->Width, frame->Height, frame->Channels, *frame->Dtype, *frame->ColorOrder);
        SendBinaryResponse(socket, 200, TEXT("OK"), TEXT("application/octet-stream"), frame->Raw, headers);
    }
}

std::shared_ptr<FCameraStreamServer::FSource> FCameraStreamServer::FindSource(const FString& path) const
{
    for (const std::shared_ptr<FSource>& source : Sources_) {
        if (path == source->StreamPath || path == source->RawPath || path == source->SnapshotPath)
            return source;
    }
    return nullptr;
}

FString FCameraStreamServer::BuildInventoryJson() const
{
    FString json = TEXT("{\"streams\":[");
    for (size_t index = 0; index < Sources_.size(); ++index) {
        const std::shared_ptr<FSource>& source = Sources_[index];
        if (index > 0)
            json += TEXT(",");
        json += FString::Printf(
            TEXT("{\"vehicle_name\":\"%s\",\"camera_name\":\"%s\",\"image_type\":%d,\"image_type_name\":\"%s\",\"annotation_name\":\"%s\",\"stream_url\":\"%s\",\"snapshot_url\":\"%s\",\"raw_url\":\"%s\"}"),
            *JsonEscape(source->VehicleName), *JsonEscape(source->CameraName), source->ImageType,
            *JsonEscape(source->ImageTypeLabel), *JsonEscape(source->AnnotationName),
            *JsonEscape(source->StreamPath), *JsonEscape(source->SnapshotPath), *JsonEscape(source->RawPath));
    }
    json += FString::Printf(
        TEXT("],\"port\":%d,\"target_fps\":%.3f,\"max_connections\":%d}"),
        Settings_.port, Settings_.target_fps, Settings_.max_connections);
    return json;
}

FString FCameraStreamServer::BuildStatusJson() const
{
    FString json = TEXT("{\"streams\":[");
    for (size_t index = 0; index < Sources_.size(); ++index) {
        const std::shared_ptr<FSource>& source = Sources_[index];
        std::shared_ptr<const FFrame> frame;
        FString error;
        {
            std::lock_guard<std::mutex> lock(source->FrameMutex);
            frame = source->LatestFrame;
            error = source->LastError;
        }
        if (index > 0)
            json += TEXT(",");
        json += FString::Printf(
            TEXT("{\"stream_url\":\"%s\",\"subscribers\":%d,\"sequence\":%llu,\"fps\":%.3f,\"latency_ms\":%.2f,\"width\":%d,\"height\":%d,\"error\":\"%s\"}"),
            *JsonEscape(source->StreamPath), source->Subscribers.load(), frame ? frame->Sequence : 0,
            frame ? frame->Fps : 0.0, frame ? frame->LatencyMs : 0.0,
            frame ? frame->Width : 0, frame ? frame->Height : 0, *JsonEscape(error));
    }
    json += TEXT("]}");
    return json;
}

FString FCameraStreamServer::BuildDashboardHtml() const
{
    return TEXT(R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AirSim Native Camera Host</title><style>
:root{color-scheme:dark;font-family:Inter,system-ui,sans-serif;background:#080b10;color:#edf2f7}*{box-sizing:border-box}body{margin:0}header{height:76px;border-bottom:1px solid #25303c;display:flex;align-items:center;justify-content:space-between;padding:0 26px;background:#0c1118}h1{font-size:17px;margin:0}header p{font-size:11px;color:#8b98a9;margin:5px 0 0}.live{color:#3ee5c3;font:12px monospace}.bar{display:flex;gap:16px;align-items:center;padding:13px 26px;border-bottom:1px solid #25303c;position:sticky;top:0;background:#0a0e14e8;z-index:4}.bar input{padding:8px 10px;background:#131a23;border:1px solid #2a3543;border-radius:7px;color:#fff}.bar label{font-size:12px;color:#9aa5b4}.grid{padding:22px 26px;display:grid;grid-template-columns:repeat(auto-fill,minmax(330px,1fr));gap:15px}.card{border:1px solid #27313e;border-radius:12px;background:#10161e;overflow:hidden;cursor:pointer}.card:focus-visible{outline:2px solid #3ee5c3;outline-offset:2px}.card.focus{grid-column:1/-1}.grid.has-focus .card:not(.focus){display:none}.frame{aspect-ratio:16/9;background:#05080c;position:relative}.frame img{width:100%;height:100%;object-fit:contain}.fps{position:absolute;right:9px;top:9px;padding:5px 7px;border:1px solid #ffffff18;border-radius:5px;background:#05080bd8;color:#3ee5c3;font:700 11px monospace}body.no-fps .fps{display:none}.meta{padding:12px 14px;display:flex;justify-content:space-between;gap:10px}.name{font:600 12px monospace}.vehicle{color:#8995a5;font-size:10px;margin-top:4px}.links a{color:#3ee5c3;font-size:10px;margin-left:10px;text-decoration:none}@media(max-width:600px){.grid{padding:14px;grid-template-columns:1fr}.bar{padding:12px 14px;flex-wrap:wrap}}
</style></head><body><header><div><h1>AirSim Native Camera Host</h1><p id="summary">Loading hosted camera routes…</p></div><div class="live">● NATIVE</div></header><div class="bar"><input id="search" placeholder="Filter vehicle or camera"><label><input id="fps" type="checkbox" checked> Real FPS</label><span id="focus-help" style="color:#7f8b9a;font-size:10px">Click a camera to focus</span></div><main id="grid" class="grid"></main><script>
let inventory={streams:[]},status={streams:[]},focusedPath=null;
const grid=document.querySelector('#grid'),search=document.querySelector('#search'),focusHelp=document.querySelector('#focus-help');
function disconnectImage(img){if(img.hasAttribute('src'))img.removeAttribute('src')}
function updateSubscriptions(){
  const focused=focusedPath!==null;
  grid.classList.toggle('has-focus',focused);
  grid.querySelectorAll('.card[data-card-path]').forEach(card=>{
    const active=!focused||card.dataset.cardPath===focusedPath;
    card.classList.toggle('focus',focused&&active);
    const img=card.querySelector('img[data-stream-url]');
    if(active){if(img.getAttribute('src')!==img.dataset.streamUrl)img.setAttribute('src',img.dataset.streamUrl)}
    else disconnectImage(img);
  });
  focusHelp.textContent=focused?'Focused · only this camera is streaming · click again to show all':'Click a camera to focus';
}
function setFocus(path){focusedPath=focusedPath===path?null:path;updateSubscriptions()}
function draw(){
  const q=search.value.toLowerCase();
  const visible=inventory.streams.filter(s=>(s.vehicle_name+' '+s.camera_name+' '+s.image_type_name).toLowerCase().includes(q));
  if(focusedPath&&!visible.some(s=>s.stream_url===focusedPath))focusedPath=null;
  grid.querySelectorAll('img[data-stream-url]').forEach(disconnectImage);
  grid.replaceChildren();
  visible.forEach(s=>{
    const card=document.createElement('article');card.className='card';card.dataset.cardPath=s.stream_url;card.tabIndex=0;
    const activate=e=>{if(e.target.closest&&e.target.closest('a'))return;if(e.type==='keydown'&&e.key!=='Enter'&&e.key!==' ')return;e.preventDefault();setFocus(s.stream_url)};
    card.onclick=activate;card.onkeydown=activate;
    const frame=document.createElement('div');frame.className='frame';
    const img=document.createElement('img');img.dataset.streamUrl=s.stream_url;img.alt=s.camera_name+' '+s.image_type_name;
    const fps=document.createElement('span');fps.className='fps';fps.dataset.statusPath=s.stream_url;fps.textContent='— FPS';frame.append(img,fps);
    const meta=document.createElement('div');meta.className='meta';const names=document.createElement('div');names.innerHTML='<div class="name"></div><div class="vehicle"></div>';names.children[0].textContent=s.camera_name+' · '+s.image_type_name;names.children[1].textContent=s.vehicle_name||'default';
    const links=document.createElement('div');links.className='links';for(const [label,url] of [['JPEG',s.snapshot_url],['RAW',s.raw_url]]){const a=document.createElement('a');a.textContent=label;a.href=url;a.target='_blank';links.append(a)}
    meta.append(names,links);card.append(frame,meta);grid.append(card)
  });
  updateSubscriptions();
}
async function poll(){try{status=await fetch('/api/status',{cache:'no-store'}).then(r=>r.json());document.querySelectorAll('.fps[data-status-path]').forEach(e=>{const s=status.streams.find(x=>x.stream_url===e.dataset.statusPath);e.textContent=s?(s.error?'ERR':Number(s.fps).toFixed(1)+' FPS'):'— FPS';if(s?.error)e.title=s.error})}catch(e){}}
fetch('/api/cameras').then(r=>r.json()).then(x=>{inventory=x;document.querySelector('#summary').textContent=x.streams.length+' settings-enabled camera/image routes · port '+x.port;draw();poll();setInterval(poll,1000)});
search.oninput=draw;document.querySelector('#fps').onchange=e=>document.body.classList.toggle('no-fps',!e.target.checked);addEventListener('pagehide',()=>grid.querySelectorAll('img[data-stream-url]').forEach(disconnectImage));
</script></body></html>)HTML");
}

FString FCameraStreamServer::ImageTypeName(int32 image_type)
{
    switch (static_cast<ImageType>(image_type)) {
    case ImageType::Scene: return TEXT("Scene");
    case ImageType::DepthPlanar: return TEXT("DepthPlanar");
    case ImageType::DepthPerspective: return TEXT("DepthPerspective");
    case ImageType::DepthVis: return TEXT("DepthVis");
    case ImageType::DisparityNormalized: return TEXT("DisparityNormalized");
    case ImageType::Segmentation: return TEXT("Segmentation");
    case ImageType::SurfaceNormals: return TEXT("SurfaceNormals");
    case ImageType::Infrared: return TEXT("Infrared");
    case ImageType::OpticalFlow: return TEXT("OpticalFlow");
    case ImageType::OpticalFlowVis: return TEXT("OpticalFlowVis");
    case ImageType::Lighting: return TEXT("Lighting");
    case ImageType::Annotation: return TEXT("Annotation");
    default: return FString::Printf(TEXT("ImageType%d"), image_type);
    }
}

FString FCameraStreamServer::JsonEscape(const FString& value)
{
    return EscapeJson(value);
}

FString FCameraStreamServer::UrlEncodeSegment(const FString& value)
{
    FTCHARToUTF8 utf8(*value);
    FString result;
    const char* data = utf8.Get();
    for (int32 index = 0; index < utf8.Length(); ++index) {
        const uint8 byte = static_cast<uint8>(data[index]);
        const bool unreserved = (byte >= 'a' && byte <= 'z') || (byte >= 'A' && byte <= 'Z') ||
                                (byte >= '0' && byte <= '9') || byte == '-' || byte == '_' || byte == '.' || byte == '~';
        if (unreserved)
            result.AppendChar(static_cast<TCHAR>(byte));
        else
            result += FString::Printf(TEXT("%%%02X"), byte);
    }
    return result;
}

uint64 FCameraStreamServer::ParseAfterSequence(
    const FString& target,
    FString& out_path,
    bool& out_has_after_sequence,
    bool& out_is_valid)
{
    out_has_after_sequence = false;
    out_is_valid = true;
    FString query;
    if (!target.Split(TEXT("?"), &out_path, &query)) {
        out_path = target;
        return 0;
    }
    TArray<FString> fields;
    query.ParseIntoArray(fields, TEXT("&"), true);
    for (const FString& field : fields) {
        FString key;
        FString value;
        if (field.Split(TEXT("="), &key, &value) && key == TEXT("after")) {
            out_has_after_sequence = true;
            if (value.IsEmpty()) {
                out_is_valid = false;
                return 0;
            }

            uint64 result = 0;
            for (const TCHAR character : value) {
                if (character < TEXT('0') || character > TEXT('9')) {
                    out_is_valid = false;
                    return 0;
                }
                const uint64 digit = static_cast<uint64>(character - TEXT('0'));
                if (result > (std::numeric_limits<uint64>::max() - digit) / 10) {
                    out_is_valid = false;
                    return 0;
                }
                result = result * 10 + digit;
            }
            return result;
        }
    }
    return 0;
}
