#include "Display.hpp"

Display::Display() {
    screen.setPenColor(vex::black);
    screen.clearScreen();
    Display::onUserCreate();
}

void Display::butonZoom(VexLib::Pose2D pos, int btnHeight, int btnWidth) {
    int buttonX = pos.getX(VexLib::DistanceUnits::mm);
    int buttonY = pos.getY(VexLib::DistanceUnits::mm);
    vex::color btnColor = flagZoom ? vex::color::green : vex::color::red;

    if (mPressed) {
        int pressX = Brain.Screen.xPosition();
        int pressY = Brain.Screen.yPosition();

        if (pressX >= buttonX && pressX <= buttonX + btnWidth &&
            pressY >= buttonY && pressY <= buttonY + btnHeight) {
            flagZoom = !flagZoom;
            btnColor = flagZoom ? vex::color::green : vex::color::red;

            if(flagZoom) {
                flagZoomOut = false; // Disable zoom out when zooming in
            }
        }
    }

    Brain.Screen.drawRectangle(buttonX, buttonY, btnWidth, btnHeight, btnColor);
    Brain.Screen.setFillColor(btnColor);
    Brain.Screen.printAt(pos.getX(VexLib::DistanceUnits::mm) + btnWidth / 2 - 10, pos.getY(VexLib::DistanceUnits::mm) + btnHeight / 2, "Zum");

    ///ZOOM OUT BUTTON
    int zoomOutX = pos.getX(VexLib::DistanceUnits::mm);
    int zoomOutY = pos.getY(VexLib::DistanceUnits::mm) + btnHeight + 10; // Position below the zoom button
    vex::color zoomOutColor = flagZoomOut ? vex::color::green : vex::color::red;

    if (mPressed) {
        int pressX = Brain.Screen.xPosition();
        int pressY = Brain.Screen.yPosition();

        if (pressX >= zoomOutX && pressX <= zoomOutX + btnWidth &&
            pressY >= zoomOutY && pressY <= zoomOutY + btnHeight) {
            flagZoomOut = !flagZoomOut;
            zoomOutColor = flagZoomOut ? vex::color::green : vex::color::red;

            if(flagZoomOut) {
                flagZoom = false; // Disable zoom in when zooming out
            }
        }
    }

    Brain.Screen.drawRectangle(zoomOutX, zoomOutY, btnWidth, btnHeight, zoomOutColor);
    Brain.Screen.setFillColor(zoomOutColor);
    Brain.Screen.printAt(zoomOutX + btnWidth / 2 - 10, zoomOutY + btnHeight / 2, "Zom");
}

void Display::WorldToScreen(float worldX, float worldY, int &screenX, int &screenY, VexLib::DistanceUnits unit) {
    screenX = (int)((worldX - offsetX) * scaleX);
    screenY = (int)((worldY - offsetY) * scaleY);
}

void Display::ScreenToWorld(int screenX, int screenY, float &worldX, float &worldY, VexLib::DistanceUnits unit) {
    worldX = (float)(screenX) / scaleX + offsetX;
    worldY = (float)(screenY) / scaleY + offsetY;
}

bool Display::onCursorUpdate(float elapsedTime) {
    float pressX = (float)Brain.Screen.xPosition();
    float pressY = (float)Brain.Screen.yPosition();

    mHold = Brain.Screen.pressing();
    mPressed = mHold && !previousMouseHeld;
    previousMouseHeld = mHold;

    if (mPressed && !flagZoom && !flagZoomOut) {
        startPanX = pressX;
        startPanY = pressY;
    } 
    if (mHold && !mPressed && !flagZoom && !flagZoomOut) {
        offsetX -= (pressX - startPanX);
        offsetY -= (pressY - startPanY);
        ///update
        startPanX = pressX;
        startPanY = pressY;
       // printf("Hold Press: offsetX: %f, offsetY: %f\n", offsetX, offsetY);
    }

    float mouseWorldX_beforeZoom, mouseWorldY_beforeZoom;
    ScreenToWorld((int)pressX, (int)pressY, mouseWorldX_beforeZoom, mouseWorldY_beforeZoom);
    if(flagZoom && mPressed){
        scaleX += 0.25f;
        scaleY += 0.25f;
    }
    if(flagZoomOut && mPressed){
        scaleX -= 0.25f;
        scaleY -= 0.25f;
    }

    float mouseWorldX_afterZoom, mouseWorldY_afterZoom;
    ScreenToWorld((int)pressX, (int)pressY, mouseWorldX_afterZoom, mouseWorldY_afterZoom);

    offsetX += (mouseWorldX_beforeZoom - mouseWorldX_afterZoom);
    offsetY += (mouseWorldY_beforeZoom - mouseWorldY_afterZoom);

    return true;
}

/*
    @Brief foloseste coord din stanga sus si lungimea + latimea 
    sa deseneze dreptunghiul de culoarea color 
*/
void Display::drawRectangle1(int x, int y, int width, int height, vex::color color) {
    screen.clearScreen(); ///POATE E REDUNDANT, am adaugat direct in userUpdate clearScreen
    screen.setPenColor(color);
    screen.drawRectangle(x, y, width, height);
    screen.setFillColor(color);
}

/*
    @Brief foloseste coord din stanga sus si coord din dreapta jos 
    sa deseneze dreptunghiul de culoarea color
*/
void Display::drawRectangle2(int x1, int y1, int x2, int y2, vex::color color) {
    int width = x2 - x1;
    int height = y2 - y1;
    Display::drawRectangle1(x1, y1, width, height, color);
}

bool Display::onUserCreate() {
    offsetX = -120;
    offsetY = -60;
    return true;
}

bool Display::onUserUpdate(float elapsedTime) {
    float sx = 0, sy = 0; float lx = 5, ly = 10;
	int pixel_sx, pixel_sy; int pixel_lx, pixel_ly;

    sx = 100; sy = 100;
    WorldToScreen(sx, sy, pixel_sx, pixel_sy);
    WorldToScreen(lx, ly, pixel_lx, pixel_ly);
    
    //screen.clearScreen(); /// CU DOUBLE BUFFERING NU MAI E NEVOIE DE CLEARSCREEN
    drawRectangle2(pixel_sx, pixel_sy, pixel_lx, pixel_ly, vex::color::blue);
    
    onCursorUpdate(elapsedTime);
    return true;
}