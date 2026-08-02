#pragma once

#include "CoreMinimal.h"
#include "common/AirSimSettings.hpp"
#include "common/ImageCaptureBase.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

class FSocket;
class FTcpListener;
class IImageWrapperModule;
class APIPCamera;
class UnrealImageCapture;
struct FIPv4Endpoint;

struct FAirSimHostedCamera
{
    FString VehicleName;
    FString CameraName;
    FString AnnotationName;
    int32 ImageType = 0;
    APIPCamera* Camera = nullptr;
    const UnrealImageCapture* ImageCapture = nullptr;
};

/**
 * Settings-driven native HTTP camera host.
 *
 * One producer is created per vehicle and batches every currently subscribed
 * camera/image-type into one UnrealImageCapture call. Multiple HTTP clients
 * share the producer's latest encoded and exact frame.
 */
class FCameraStreamServer
{
public:
    FCameraStreamServer(
        const msr::airlib::AirSimSettings::CameraHostSetting& Settings,
        std::vector<FAirSimHostedCamera> HostedCameras);
    ~FCameraStreamServer();

    bool Start();
    void Stop();
    bool IsRunning() const;

private:
    struct FFrame;
    struct FSource;
    struct FHttpRequest;
    struct FClientThread;
    class FCaptureWorker;

    bool HandleAcceptedConnection(FSocket* Socket, const FIPv4Endpoint& RemoteEndpoint);
    void HandleConnection(FSocket* Socket);
    void FinishConnection(FSocket* Socket);

    bool ReadRequest(FSocket* Socket, FHttpRequest& OutRequest) const;
    bool SendAll(FSocket* Socket, const uint8* Data, int64 Size) const;
    bool SendTextResponse(FSocket* Socket, int32 StatusCode, const TCHAR* StatusText, const FString& ContentType, const FString& Body) const;
    bool SendBinaryResponse(
        FSocket* Socket,
        int32 StatusCode,
        const TCHAR* StatusText,
        const FString& ContentType,
        const std::shared_ptr<const std::vector<uint8>>& Body,
        const FString& ExtraHeaders = FString()) const;

    void ServeMjpeg(FSocket* Socket, const std::shared_ptr<FSource>& Source);
    void ServeJpeg(FSocket* Socket, const std::shared_ptr<FSource>& Source, uint64 AfterSequence, bool RequireFreshFrame);
    void ServeRaw(FSocket* Socket, const std::shared_ptr<FSource>& Source, uint64 AfterSequence, bool RequireFreshFrame);
    void ServeGimbalInventory(FSocket* Socket);
    void ServeGimbalCommand(FSocket* Socket, const FString& Body);
    std::shared_ptr<const FFrame> WaitForFrame(
        const std::shared_ptr<FSource>& Source,
        uint64 AfterSequence,
        double TimeoutSeconds,
        bool RequireJpeg) const;
    uint64 BeginSubscription(
        const std::shared_ptr<FSource>& Source,
        uint64 AfterSequence,
        bool RequireFreshFrame,
        bool RequireJpeg);
    void EndSubscription(const std::shared_ptr<FSource>& Source, bool RequireJpeg);
    void ReapClientThreadsLocked();

    FString BuildInventoryJson() const;
    FString BuildStatusJson() const;
    FString BuildDashboardHtml() const;
    std::shared_ptr<FSource> FindSource(const FString& Path) const;
    std::shared_ptr<FSource> FindCameraSource(const FString& VehicleName, const FString& CameraName) const;

    static FString ImageTypeName(int32 ImageType);
    static FString JsonEscape(const FString& Value);
    static FString UrlEncodeSegment(const FString& Value);
    static uint64 ParseAfterSequence(
        const FString& Target, FString& OutPath, bool& OutHasAfterSequence, bool& OutIsValid);

private:
    msr::airlib::AirSimSettings::CameraHostSetting Settings_;
    std::atomic<bool> Running_{ false };
    IImageWrapperModule* ImageWrapperModule_ = nullptr;
    TUniquePtr<FTcpListener> Listener_;

    std::vector<std::shared_ptr<FSource>> Sources_;
    std::vector<std::unique_ptr<FCaptureWorker>> CaptureWorkers_;

    mutable std::mutex ClientsMutex_;
    std::set<FSocket*> ClientSockets_;
    std::vector<std::shared_ptr<FClientThread>> ClientThreads_;
};
