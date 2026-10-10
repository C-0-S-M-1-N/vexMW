#include "Display.hpp"
#include <fstream>

std::ifstream fisier("date.txt");

///X Y A si B pentru a deschide meniul burger pentru zoom si tastatura

Display::Display() {
    screen.setPenColor(vex::white);
    screen.clearScreen();
}

bool combo = false;

void Display::butonZoom(VexLib::Pose2D pos, int btnHeight, int btnWidth) {
    int buttonX = pos.getX(VexLib::DistanceUnits::mm);
    int buttonY = pos.getY(VexLib::DistanceUnits::mm);
    vex::color btnColor = flagZoom ? vex::color::green : vex::color::red;

    Brain.Screen.drawRectangle(buttonX, buttonY, btnWidth, btnHeight, btnColor);
    Brain.Screen.setFillColor(btnColor);
    Brain.Screen.printAt(pos.getX(VexLib::DistanceUnits::mm) + btnWidth / 2 - 10, pos.getY(VexLib::DistanceUnits::mm) + btnHeight / 2, "+");

    ///ZOOM OUT BUTTON
    int zoomOutX = pos.getX(VexLib::DistanceUnits::mm);
    int zoomOutY = pos.getY(VexLib::DistanceUnits::mm) + btnHeight + 10;
    vex::color zoomOutColor = flagZoomOut ? vex::color::green : vex::color::red;

    Brain.Screen.drawRectangle(zoomOutX, zoomOutY, btnWidth, btnHeight, zoomOutColor);
    Brain.Screen.setFillColor(zoomOutColor);
    Brain.Screen.printAt(zoomOutX + btnWidth / 2 - 10, zoomOutY + btnHeight / 2, "-");
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

    bool zoomButtonPressed = false;
    if (mPressed && combo) {
        const int buttonX = 20;
        const int buttonY = 20;
        const int buttonWidth = 60;
        const int buttonHeight = 80;

        if (pressX >= buttonX && pressX <= buttonX + buttonWidth &&
            pressY >= buttonY && pressY <= buttonY + buttonHeight) {
            flagZoom = !flagZoom;
            if (flagZoom) {
                flagZoomOut = false;
            }
            zoomButtonPressed = true;
        } else {
            const int zoomOutY = buttonY + buttonHeight + 10;
            if (pressX >= buttonX && pressX <= buttonX + buttonWidth &&
                pressY >= zoomOutY && pressY <= zoomOutY + buttonHeight) {
                flagZoomOut = !flagZoomOut;
                if (flagZoomOut) {
                    flagZoom = false;
                }
                zoomButtonPressed = true;
            }
        }
    }

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
        //printf("Hold Press: offsetX: %f, offsetY: %f\n", offsetX, offsetY);
    }

    float mouseWorldX_beforeZoom, mouseWorldY_beforeZoom;
    ScreenToWorld((int)pressX, (int)pressY, mouseWorldX_beforeZoom, mouseWorldY_beforeZoom);
    if (flagZoom && mPressed && !zoomButtonPressed) {
        scaleX += 0.125f;
        scaleY += 0.125f;
    }
    if (flagZoomOut && mPressed && !zoomButtonPressed) {
        scaleX = scaleX > 0.125f ? scaleX - 0.125f : 0.125f;
        scaleY = scaleY > 0.125f ? scaleY - 0.125f : 0.125f;
    }

    float mouseWorldX_afterZoom, mouseWorldY_afterZoom;
    ScreenToWorld((int)pressX, (int)pressY, mouseWorldX_afterZoom, mouseWorldY_afterZoom);

    offsetX += (mouseWorldX_beforeZoom - mouseWorldX_afterZoom);
    offsetY += (mouseWorldY_beforeZoom - mouseWorldY_afterZoom);

    return true;
}

void Display::clearScreen(){
    screen.clearScreen();
}

float f[5] = {100, -50, 60, 200, 150};
float t[5] = {0, 240, 250, 300, 380};
float t2[100];
float f2[100];

void Display::renderScreen(){
    clearScreen();
    drawAxes();
        
    if (combo) {  ///combo se comporta ca un switch ptr. Hamburger
            butonZoom(VexLib::Pose2D(5, 5), 30, 30);
            tastatura();
    } else {
        flagZoom = false;
        flagZoomOut = false;
        tastaturaPreviousVisible = false;
        tastaturaPreviousPressing = false;
    }
    for(int i = 0; i < 100; i++){
        t2[i] = i*4;
        f2[i] = 50 * sin(i * 0.1);
    }
    drawFunction(f2, t2, 100, vex::color::red);
    //drawFunction(f, t, 5, vex::color::yellow);

    //screen.render(true);
}

void Display::drawRectangle(int x, int y, int width, int height, vex::color color) {
	int pixel_sx, pixel_sy; int pixel_lx, pixel_ly;

    WorldToScreen(x, y, pixel_sx, pixel_sy);
    
    int rectangleWidth = (int)(std::abs(width * scaleX));
    int rectangleHeight = (int)(std::abs(height * scaleY));

    screen.setPenColor(color);
    screen.setFillColor(color);
    screen.drawRectangle(pixel_sx, pixel_sy, rectangleWidth, rectangleHeight);
}

void Display::drawCircle(int xCentre, int yCentre, int radius, vex::color color){
    int pixel_xCentre, pixel_yCentre, pixel_radius, temp;
    WorldToScreen(xCentre, yCentre, pixel_xCentre, pixel_yCentre);

    int printingRadius = (int)(std::abs(radius * scaleX)); // doar daca scaleX = scaleY

    screen.setPenColor(color);
    screen.setFillColor(color);
    screen.drawCircle(pixel_xCentre, pixel_yCentre, printingRadius);

}

void Display::drawLine(float sx1, float sy1, float sx2, float sy2, vex::color color){
    int pixel_sx1, pixel_sx2, pixel_sy1, pixel_sy2;
    WorldToScreen(sx1, sy1, pixel_sx1, pixel_sy1);
    WorldToScreen(sx2, sy2, pixel_sx2, pixel_sy2);

    screen.setPenColor(color);
    screen.drawLine(pixel_sx1, pixel_sy1, pixel_sx2, pixel_sy2);
}


bool Display::onUserCreate() {
    offsetX = -60;
    offsetY = -60;
    return true;
}

float dimensiuneAxaX = 960, dimensiuneAxaY = 270;
void Display::drawAxes(){
    float sx1 = 0, sy1 =  135 - dimensiuneAxaY / 2;
    float sx2 = 0, sy2 = dimensiuneAxaY;
    drawLine(sx1, sy1, sx2, sy2, vex::color::white);
    float sx1_2 =-60, sy1_2 = dimensiuneAxaY / 2;
    float sx2_2 = dimensiuneAxaX, sy2_2 = dimensiuneAxaY / 2;
    drawLine(sx1_2, sy1_2, sx2_2, sy2_2, vex::color::white);
}

float scaleFctY, scaleFctX;
void Display::drawFunction(float (*func), float (*time), int lungime, vex::color color){
    float maxTime = time[lungime - 1];
    if(maxTime > dimensiuneAxaX){
        dimensiuneAxaX = maxTime + 40;
    }
    scaleFctX = scaleFctY = 480.0f / maxTime;
    for(int i = 0; i < lungime - 1; i++){
        float t1, t2, y1, y2;
        t1 = time[i];
        t2 = time[i + 1];
        y1 = func[i];
        y2 = func[i + 1];
        if(abs(y1) * 2 > dimensiuneAxaY){ //|| abs(y2) * 2 > dimensiuneAxaY){
            dimensiuneAxaY = abs(y1) * 2 + 40;  //cu surplus de 20 pe fiecare cadran
        }
        if(maxTime < 480){
            t1 = t1 * scaleFctX;
            t2 = t2 * scaleFctX;
            y2 = y2 * scaleFctY;
            y1 = y1 * scaleFctY;
        }
        Display::drawLine(t1, -y1 + dimensiuneAxaY / 2, t2, -y2 + dimensiuneAxaY / 2, color);
    }
}


//---------------------------

bool Display::onUserUpdate(float elapsedTime, vex::controller cntrler) {
    if (cntrler.ButtonX.pressing() && cntrler.ButtonY.pressing() &&
        cntrler.ButtonA.pressing() && cntrler.ButtonB.pressing()) {
        if (!flagHamburger) {
            combo = !combo;
            flagHamburger = true;
        }
    } else {
        flagHamburger = false;
    }

    onCursorUpdate(elapsedTime);
    return true;
}

void Display::tastatura(){

    const int keyboardX = 230;
    const int keyboardY = 25;
    const int buttonWidth = 55;
    const int buttonHeight = 40;
    const int gap = 5;
    const char* keys[4][4] = {
        {"7", "8", "9", "<"},
        {"4", "5", "6", ""},
        {"1", "2", "3", ""},
        {".", "0", "Ent", ""}
    };

    if (!tastaturaPreviousVisible) {
        tastaturaText.clear();
        tastaturaEnterPressed = false;
    }

    int pressX = screen.xPosition();
    int pressY = screen.yPosition();
    bool pressing = screen.pressing();
    bool pressed = pressing && !tastaturaPreviousPressing;

    if (pressed) {
        for (int row = 0; row < 4; row++) {
            for (int column = 0; column < 4; column++) {
                const std::string key = keys[row][column];
                if (key.empty()) continue;

                int buttonX = keyboardX + column * (buttonWidth + gap);
                int buttonY = keyboardY + row * (buttonHeight + gap);
                bool insideButton = (pressX >= buttonX && pressX <= buttonX + buttonWidth &&
                                     pressY >= buttonY && pressY <= buttonY + buttonHeight);

                if (!insideButton) {
                    continue;
                }

                if (key == ".") {
                    if (tastaturaText.find('.') == std::string::npos) {
                        tastaturaText += key;
                    }
                } else if (key == "Ent") {
                    tastaturaEnterPressed = true;
                    combo = false;
                } else if (key == "<") {
                    if (!tastaturaText.empty()) {
                        tastaturaText.pop_back();
                    }
                } else {
                    tastaturaText += key;
                    tastaturaEnterPressed = false;
                }
            }
        }
    }

    screen.setPenColor(vex::color::white);
    screen.setFillColor(vex::color::black);
    screen.drawRectangle(keyboardX - gap, keyboardY - 22, 4 * buttonWidth + 5 * gap, 4 * buttonHeight + 3 * gap + 30);
    screen.printAt(keyboardX, keyboardY - 5, false, "%s", tastaturaText.c_str());

    for (int row = 0; row < 4; row++) {
        for (int column = 0; column < 4; column++) {
            const std::string key = keys[row][column];
            if (key.empty()) continue;

            int buttonX = keyboardX + column * (buttonWidth + gap);
            int buttonY = keyboardY + row * (buttonHeight + gap);
            screen.setFillColor(vex::color::blue);
            screen.drawRectangle(buttonX, buttonY, buttonWidth, buttonHeight);
            screen.setPenColor(vex::color::white);
            screen.printAt(buttonX + buttonWidth / 2 - 5, buttonY + buttonHeight / 2 + 5, false, "%s", keys[row][column]);
        }
    }

    tastaturaPreviousPressing = pressing;
    tastaturaPreviousVisible = true;

}

const std::string& Display::getTastaturaText() const {
    return tastaturaText;
}

bool Display::tastaturaAConfirmat() const {
    return tastaturaEnterPressed;
}