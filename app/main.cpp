// app/main.cpp — entry point
#include "app.h"
#include <string>
#include <windows.h>

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR cmdLine, int) {
    std::string engineJson = "assets/engines/baseline_i4.json";
    if (cmdLine && *cmdLine) {
        engineJson = cmdLine;
    }

    ice::App app;
    if (!app.init(engineJson)) {
        return 1;
    }
    app.run();
    app.shutdown();
    return 0;
}

#ifndef _CONSOLE
int main() {
    char empty[] = "";
    return WinMain(GetModuleHandle(nullptr), nullptr, empty, SW_SHOWDEFAULT);
}
#endif
