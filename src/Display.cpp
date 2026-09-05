#include "Display.hpp"

Display::Display() {
    screen.setPenColor(vex::white);
    screen.clearScreen();
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

void Display::drawRectangle(int x, int y, int width, int height, vex::color color) {
    screen.clearScreen(); ///POATE E REDUNDANT, am adaugat direct in userUpdate clearScreen
    screen.setPenColor(color);
    screen.drawRectangle(x, y, width, height);
    screen.setFillColor(color);
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
    int rectangleX = pixel_sx < pixel_lx ? pixel_sx : pixel_lx; ///stie geminy sa converteasca coordonatele in marimi
    int rectangleY = pixel_sy < pixel_ly ? pixel_sy : pixel_ly;
    int rectangleWidth = pixel_sx < pixel_lx ? pixel_lx - pixel_sx : pixel_sx - pixel_lx;
    int rectangleHeight = pixel_sy < pixel_ly ? pixel_ly - pixel_sy : pixel_sy - pixel_ly;
    drawRectangle(rectangleX, rectangleY, rectangleWidth, rectangleHeight, vex::color::blue);
    
    onCursorUpdate(elapsedTime);
    return true;
}