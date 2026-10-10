#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include "vex.h"
#include "vex_brain.h"
#include "VexLib.hpp"
#include <string>

extern vex::brain Brain;

class Display {
private:
    vex::brain::lcd& screen = Brain.Screen;
    bool flagZoom = false; bool flagZoomOut = false; bool flagHamburger = false;
    unsigned short int flags = 0x00;

    float offsetX = 0.0f; float offsetY = 0.0f;
    float startPanX = 0.0f; float startPanY = 0.0f;
    float scaleX = 1.0f; float scaleY = 1.0f;

    std::string tastaturaText;
    bool tastaturaPreviousPressing = false;
    bool tastaturaPreviousVisible = false;
    bool tastaturaEnterPressed = false;

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
    void drawCircle(int xCentre, int yCentre, int radius, vex::color color);
    void drawLine(float sx1, float sy1, float sx2, float sy2, vex::color color);
    void drawAxes();
    //void drawFunction(float (*func)(float), float (*time)(float), vex::color color);
    void drawFunction(float (*func), float (*time), int lungime, vex::color color);
    void clearScreen();
    void renderScreen();
    virtual bool onUserCreate();
    virtual bool onUserUpdate(float elapsedTime, vex::controller cntrler);

    void tastatura();
    const std::string& getTastaturaText() const;
    bool tastaturaAConfirmat() const;
};

#endif
