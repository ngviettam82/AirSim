#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "PIPCamera.h"
#include <functional>
#include "SimHUDWidget.generated.h"

class UCanvasPanelSlot;
class UTextureRenderTarget2D;
class UWidget;

UCLASS()
class AIRSIM_API USimHUDWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "Event handler")
    void onToggleRecordingButtonClick();

public:
    typedef std::function<void(void)> OnToggleRecording;

    //TODO: Tick is not working
    //virtual void Tick_Implementation(FGeometry MyGeometry, float InDeltaTime) override;

    void updateDebugReport(const std::string& text);
    void setReportVisible(bool is_visible);
    void toggleHelpVisibility();

    void setOnToggleRecordingHandler(OnToggleRecording handler);
    void setSubwindowRenderTargetSize(int window_index, UTextureRenderTarget2D* render_target);

public:
    //below are implemented in Blueprint. The return value is forced to be
    //bool even when not needed because of Unreal quirk that if return value
    //is not there then below are treated as events instead of overridable functions
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setSubwindowVisibility(int window_index, bool is_visible, UTextureRenderTarget2D* render_target);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    int getSubwindowVisibility(int window_index);

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setRecordButtonVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getRecordButtonVisibility();

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool initializeForPlay();

protected:
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setReportContainerVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getReportContainerVisibility();

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setHelpContainerVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getHelpContainerVisibility();

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setReportText(const FString& text);

private:
    FVector2D getBaseSubwindowSlotSize(const FName& widget_name, UCanvasPanelSlot* canvas_slot);
    void applySubwindowAspectSize(UWidget* widget, float aspect_ratio, bool restore);

    OnToggleRecording on_toggle_recording_;
    TMap<FName, FVector2D> subwindow_base_slot_sizes_;
};
