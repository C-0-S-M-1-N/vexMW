#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include "vex.h"
#include "vex_brain.h"
#include "VexLib.hpp"

extern vex::brain Brain;

class Display {
private:
    vex::brain::lcd& screen = Brain.Screen;
    bool flagZoom = false; bool flagZoomOut = false;

    float offsetX = 0.0f; float offsetY = 0.0f;

    float startPanX = 0.0f; float startPanY = 0.0f;

    float scaleX = 1.0f; float scaleY = 1.0f;

public:
    bool previousMouseHeld = false;
    Display();
    void butonZoom(VexLib::Pose2D pos = VexLib::Pose2D(0, 0), int btnHeight = 50, int btnWidth = 30);
    bool mHold = false;
    bool mPressed = false;

    void WorldToScreen(float worldX, float worldY, int &screenX, int &screenY, VexLib::DistanceUnits unit = VexLib::DistanceUnits::mm);
    void ScreenToWorld(int screenX, int screenY, float &worldX, float &worldY, VexLib::DistanceUnits unit = VexLib::DistanceUnits::mm);
    bool onCursorUpdate(float elapsedTime);
    void drawRectangle(int x, int y, int width, int height, vex::color color);
    virtual bool onUserCreate();
    virtual bool onUserUpdate(float elapsedTime);
};

#endif
