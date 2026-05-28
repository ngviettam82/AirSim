#include "SimHUDWidget.h"

#include "Blueprint/WidgetTree.h"
#include "Components/CanvasPanelSlot.h"
#include "Components/Image.h"
#include "Components/ScaleBox.h"
#include "Components/Widget.h"
#include "Engine/TextureRenderTarget2D.h"

namespace
{
    UWidget* findSubwindowWidget(UWidgetTree* widget_tree, int window_index, const TCHAR* suffix)
    {
        if (widget_tree == nullptr || window_index < 0) {
            return nullptr;
        }

        const FName widget_name(*FString::Printf(TEXT("SubWindow%d%s"), window_index, suffix));
        return widget_tree->FindWidget(widget_name);
    }

    FVector2D fitSizeToAspect(const FVector2D& bounds, float aspect_ratio)
    {
        if (bounds.X <= 0.0f || bounds.Y <= 0.0f || aspect_ratio <= 0.0f) {
            return bounds;
        }

        const float bounds_aspect = bounds.X / bounds.Y;
        if (bounds_aspect > aspect_ratio) {
            return FVector2D(bounds.Y * aspect_ratio, bounds.Y);
        }
        return FVector2D(bounds.X, bounds.X / aspect_ratio);
    }
}

void USimHUDWidget::updateDebugReport(const std::string& text)
{
    setReportText(FString(text.c_str()));
}

void USimHUDWidget::setReportVisible(bool is_visible)
{
    setReportContainerVisibility(is_visible);
}

void USimHUDWidget::toggleHelpVisibility()
{
    setHelpContainerVisibility(!getHelpContainerVisibility());
}

void USimHUDWidget::setOnToggleRecordingHandler(OnToggleRecording handler)
{
    on_toggle_recording_ = handler;
}

void USimHUDWidget::setSubwindowRenderTargetSize(int window_index, UTextureRenderTarget2D* render_target)
{
    UWidget* container = findSubwindowWidget(WidgetTree, window_index, TEXT("Container"));
    UWidget* scale_box_widget = findSubwindowWidget(WidgetTree, window_index, TEXT("ScaleBox"));
    UWidget* image_widget = findSubwindowWidget(WidgetTree, window_index, TEXT("Image"));

    if (UScaleBox* scale_box = Cast<UScaleBox>(scale_box_widget)) {
        scale_box->SetStretch(EStretch::ScaleToFit);
        scale_box->SetStretchDirection(EStretchDirection::Both);
    }

    const bool restore = render_target == nullptr || render_target->SizeX <= 0 || render_target->SizeY <= 0;
    float aspect_ratio = 0.0f;
    if (!restore) {
        const FVector2D render_target_size(static_cast<float>(render_target->SizeX), static_cast<float>(render_target->SizeY));
        aspect_ratio = render_target_size.X / render_target_size.Y;

        if (UImage* image = Cast<UImage>(image_widget)) {
            FSlateBrush brush = image->Brush;
            brush.ImageSize = render_target_size;
            image->SetBrush(brush);
        }
    }

    applySubwindowAspectSize(container, aspect_ratio, restore);
    applySubwindowAspectSize(scale_box_widget, aspect_ratio, restore);
    applySubwindowAspectSize(image_widget, aspect_ratio, restore);
}

FVector2D USimHUDWidget::getBaseSubwindowSlotSize(const FName& widget_name, UCanvasPanelSlot* canvas_slot)
{
    if (const FVector2D* base_size = subwindow_base_slot_sizes_.Find(widget_name)) {
        return *base_size;
    }

    const FVector2D base_size = canvas_slot != nullptr ? canvas_slot->GetSize() : FVector2D::ZeroVector;
    subwindow_base_slot_sizes_.Add(widget_name, base_size);
    return base_size;
}

void USimHUDWidget::applySubwindowAspectSize(UWidget* widget, float aspect_ratio, bool restore)
{
    if (widget == nullptr) {
        return;
    }

    UCanvasPanelSlot* canvas_slot = Cast<UCanvasPanelSlot>(widget->Slot);
    if (canvas_slot == nullptr) {
        return;
    }

    const FVector2D base_size = getBaseSubwindowSlotSize(widget->GetFName(), canvas_slot);
    if (restore || aspect_ratio <= 0.0f || base_size.X <= 0.0f || base_size.Y <= 0.0f) {
        canvas_slot->SetSize(base_size);
        return;
    }

    canvas_slot->SetSize(fitSizeToAspect(base_size, aspect_ratio));
}

void USimHUDWidget::onToggleRecordingButtonClick()
{
    on_toggle_recording_();
}
