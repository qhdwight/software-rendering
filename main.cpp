#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <common.hpp>

namespace Engine
{
    // Coordinate systems:
    //
    //  World: right-handed, +X forward,  +Y left, +Z up
    // Camera: right-handed, +Z forward, +X right, +Y down
    //    NDC: right-handed, +Z forward, +X right, +Y down, X,Y in [-1, 1], Z in [0, 1]

    static constexpr U32 kWindowWidth = 800;
    static constexpr U32 kWindowHeight = 600;
    static constexpr U32 kMaxCubes = 1024;

    constexpr F32 kAspect = static_cast<F32>(kWindowWidth) / static_cast<F32>(kWindowHeight);
    constexpr F32 kFovY = 80.0f * (3.14159265f / 180.0f);
    constexpr F32 kTanHalfFov = Tan(kFovY * 0.5f);

    struct Resources
    {
        HWND m_hWindow = nullptr;
        BITMAPINFO m_bitmapInfo;
        void *m_pixels = nullptr;
        HBITMAP m_hBitmap = nullptr;
        HDC m_hMemDC = nullptr;
        LONG m_mouseX = 0;
        LONG m_mouseY = 0;
        bool m_forward = false;
        bool m_backward = false;
        bool m_left = false;
        bool m_right = false;
    };

    struct State
    {
        bool m_isRunning = true;

        F32 m_camInWorldX;
        F32 m_camInWorldY;
        F32 m_camInWorldZ;
        F32 m_camInWorldW;
        F32 m_camInWorldE23;
        F32 m_camInWorldE13;
        F32 m_camInWorldE12;

        U32 m_numCubes = 0;
        alignas(F32x4) F32 m_cubeInWorldX[kMaxCubes];
        alignas(F32x4) F32 m_cubeInWorldY[kMaxCubes];
        alignas(F32x4) F32 m_cubeInWorldZ[kMaxCubes];
        alignas(F32x4) F32 m_cubeInWorldW[kMaxCubes];
        alignas(F32x4) F32 m_cubeInWorldE23[kMaxCubes];
        alignas(F32x4) F32 m_cubeInWorldE13[kMaxCubes];
        alignas(F32x4) F32 m_cubeInWorldE12[kMaxCubes];
        alignas(F32x4) F32 m_cubeSize[kMaxCubes];
    };

    LRESULT CALLBACK ProcessCallback(const HWND hWnd, const UINT uMsg, const WPARAM wParam, const LPARAM lParam)
    {
        Resources &resources = *reinterpret_cast<Resources *>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
        switch (uMsg)
        {
            case WM_NCCREATE: {
                const CREATESTRUCT *const pCreateStruct = reinterpret_cast<CREATESTRUCT *>(lParam);
                SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pCreateStruct->lpCreateParams));
                return true;
            }
            case WM_CREATE: {
                const HDC hDC = GetDC(hWnd);
                resources.m_bitmapInfo = {
                    .bmiHeader = {
                        .biSize = sizeof(BITMAPINFOHEADER),
                        .biWidth = kWindowWidth,
                        .biHeight = -static_cast<LONG>(kWindowHeight),
                        .biPlanes = 1,
                        .biBitCount = 32,
                        .biCompression = BI_RGB
                    },
                };
                resources.m_hBitmap = CreateDIBSection(
                    hDC,
                    &resources.m_bitmapInfo,
                    DIB_RGB_COLORS,
                    &resources.m_pixels,
                    nullptr,
                    0
                    );
                resources.m_hMemDC = CreateCompatibleDC(hDC);
                SelectObject(resources.m_hMemDC, resources.m_hBitmap);
                return 0;
            }
            case WM_DESTROY: {
                DeleteDC(resources.m_hMemDC);
                DeleteObject(resources.m_hBitmap);
                PostQuitMessage(0);
                return 0;
            }
            case WM_PAINT: {
                PAINTSTRUCT ps;
                const HDC hDC = BeginPaint(hWnd, &ps);
                BitBlt(hDC, 0, 0, kWindowWidth, kWindowHeight, resources.m_hMemDC, 0, 0, SRCCOPY);
                EndPaint(hWnd, &ps);
                return 0;
            }
            case WM_INPUT: {
                UINT dwSize = sizeof(RAWINPUT);
                RAWINPUT raw;
                GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, &raw, &dwSize, sizeof(RAWINPUTHEADER));

                if (raw.header.dwType == RIM_TYPEMOUSE)
                {
                    resources.m_mouseX += raw.data.mouse.lLastX;
                    resources.m_mouseY += raw.data.mouse.lLastY;
                }
                else if (raw.header.dwType == RIM_TYPEKEYBOARD)
                {
                    const USHORT makeCode = raw.data.keyboard.MakeCode;
                    const USHORT flags = raw.data.keyboard.Flags;
                    const bool isBreak = (flags & RI_KEY_BREAK) != 0;
                    switch (makeCode)
                    {
                        case 0x11:
                            resources.m_forward = !isBreak;
                            break;
                        case 0x1F:
                            resources.m_backward = !isBreak;
                            break;
                        case 0x1E:
                            resources.m_left = !isBreak;
                            break;
                        case 0x20:
                            resources.m_right = !isBreak;
                            break;
                        default:
                            break;
                    }
                }
                return 0;
            }
            default: {
                return DefWindowProc(hWnd, uMsg, wParam, lParam);
            }
        }
    }

    Vec3f WindowToCamera(const U32 x, const U32 y)
    {
        const F32 xInNdc = 2.0f * (static_cast<F32>(x) + 0.5f) / static_cast<F32>(kWindowWidth) - 1.0f;
        const F32 yInNdc = 1.0f - 2.0f * (static_cast<F32>(y) + 0.5f) / static_cast<F32>(kWindowHeight);
        const F32 xInCam = xInNdc * kAspect * kTanHalfFov;
        const F32 yInCam = yInNdc * kTanHalfFov;
        return Vec3f(xInCam, yInCam, 0.0f);
    }

    U32 ComputeFragment(const Vec3f pixelInCamera, const State &state, const Pose &cameraToWorld)
    {
        constexpr U32 kBackground = 0xFF111111;

        const Vec3f pixelInWorld = Transform(cameraToWorld, pixelInCamera);
        const Vec3f pixelDirInWorld = Rotate(cameraToWorld.m_ori, Vec3f(pixelInCamera[0], pixelInCamera[1], 1.0f));

        U32 pixel = kBackground;

        for (U32 iCube = 0; iCube < state.m_numCubes; ++iCube)
        {
            const Pose cubeInWorld(
                Vec3f(
                    state.m_cubeInWorldX[iCube],
                    state.m_cubeInWorldY[iCube],
                    state.m_cubeInWorldZ[iCube]
                    ),
                Quatf(
                    state.m_cubeInWorldW[iCube],
                    state.m_cubeInWorldE23[iCube],
                    state.m_cubeInWorldE13[iCube],
                    state.m_cubeInWorldE12[iCube]
                    )
                );
            const Pose worldToCube = Inverse(cubeInWorld);
            const Vec3f pointInCube = Transform(worldToCube, pixelInWorld);
            const Vec3f pixelDirInCube = Rotate(worldToCube.m_ori, pixelDirInWorld);
            const F32 hs = state.m_cubeSize[iCube] * 0.5f;
            const Vec3f minInCube(-hs, -hs, -hs);
            const Vec3f maxInCube(hs, hs, hs);
            F32 tNear = 0.0f;
            F32 tFar = 4096.0f;
            U32 hitFace = 0;
            for (U32 iAxis = 0; iAxis < 3; ++iAxis)
            {
                // TODO: Handle divide by zero...
                const F32 t1 = (minInCube[iAxis] - pointInCube[iAxis]) / pixelDirInCube[iAxis];
                const F32 t2 = (maxInCube[iAxis] - pointInCube[iAxis]) / pixelDirInCube[iAxis];
                const F32 tMin = Min(t1, t2);
                const F32 tMax = Max(t1, t2);
                if (tMin > tNear)
                {
                    tNear = tMin;
                    hitFace = iAxis * 2 + (pixelDirInCube[iAxis] < 0.0f);
                }
                if (tMax < tFar)
                {
                    tFar = tMax;
                }
                if (tNear > tFar)
                {
                    break;
                }
            }

            if (tNear < tFar)
            {
                constexpr U32 kFaceColors[] = {
                    0xFFFF0000, // X+ (red)
                    0xFF880000, // X- (dark red)
                    0xFF00FF00, // Y+ (green)
                    0xFF008800, // Y- (dark green)
                    0xFF0000FF, // Z+ (blue)
                    0xFF000088, // Z- (dark blue)
                };
                pixel = kFaceColors[hitFace];
                break;
            }
        }

        return pixel;
    }

    void HandleInput(const Resources &resources, State &state)
    {
        const Quatf camInWorld(
            state.m_camInWorldW,
            state.m_camInWorldE23,
            state.m_camInWorldE13,
            state.m_camInWorldE12
            );

        if (resources.m_mouseX)
        {
            const Quatf dQ = FromAngleAxis(0.002f * static_cast<F32>(resources.m_mouseX), Vec3f{0.0f, 1.0f, 0.0f});
            const Quatf newCamInWorld = Normalize(camInWorld * dQ);
            state.m_camInWorldW = newCamInWorld[0];
            state.m_camInWorldE23 = newCamInWorld[1];
            state.m_camInWorldE13 = newCamInWorld[2];
            state.m_camInWorldE12 = newCamInWorld[3];
        }

        const Vec3f forwardInWorld = Rotate(camInWorld, Vec3f{0.0f, 0.0f, 1.0f});
        const Vec3f rightInWorld = Rotate(camInWorld, Vec3f{1.0f, 0.0f, 0.0f});
        if (resources.m_forward)
        {
            state.m_camInWorldX += forwardInWorld[0] * 0.1f;
            state.m_camInWorldY += forwardInWorld[1] * 0.1f;
            state.m_camInWorldZ += forwardInWorld[2] * 0.1f;
        }
        if (resources.m_backward)
        {
            state.m_camInWorldX -= forwardInWorld[0] * 0.1f;
            state.m_camInWorldY -= forwardInWorld[1] * 0.1f;
            state.m_camInWorldZ -= forwardInWorld[2] * 0.1f;
        }
        if (resources.m_left)
        {
            state.m_camInWorldX -= rightInWorld[0] * 0.1f;
            state.m_camInWorldY -= rightInWorld[1] * 0.1f;
            state.m_camInWorldZ -= rightInWorld[2] * 0.1f;
        }
        if (resources.m_right)
        {
            state.m_camInWorldX += rightInWorld[0] * 0.1f;
            state.m_camInWorldY += rightInWorld[1] * 0.1f;
            state.m_camInWorldZ += rightInWorld[2] * 0.1f;
        }
    }

    void RenderFrame(const Resources &resources, const State &state)
    {
        const Pose camInWorld(
            Vec3f(
                state.m_camInWorldX,
                state.m_camInWorldY,
                state.m_camInWorldZ
                ),
            Quatf(
                state.m_camInWorldW,
                state.m_camInWorldE23,
                state.m_camInWorldE13,
                state.m_camInWorldE12
                )
            );
        U32 *pPixels = static_cast<U32 *>(resources.m_pixels);
#pragma omp parallel for
        for (U32 y = 0; y < kWindowHeight; ++y)
        {
            for (U32 x = 0; x < kWindowWidth; ++x)
            {
                const Vec3f pixelInCamera = WindowToCamera(x, y);
                pPixels[y * kWindowWidth + x] = ComputeFragment(pixelInCamera, state, camInWorld);
            }
        }
        InvalidateRect(resources.m_hWindow, nullptr, FALSE);
    }

    void Run(Resources &resources, State &state)
    {
        // Add cube.
        constexpr F32 x[] = {2.0f, 2.0f, -2.0f, -2.0f};
        constexpr F32 z[] = {2.0f, -2.0f, 2.0f, -2.0f};
        for (U32 i = 0; i < 4; ++i)
        {
            const U32 iCube = state.m_numCubes++;

            state.m_cubeInWorldX[iCube] = x[i];
            state.m_cubeInWorldY[iCube] = 0.0f;
            state.m_cubeInWorldZ[iCube] = z[i];

            state.m_cubeInWorldW[iCube] = 1.0f;
            state.m_cubeInWorldE23[iCube] = 0.0f;
            state.m_cubeInWorldE13[iCube] = 0.0f;
            state.m_cubeInWorldE12[iCube] = 0.0f;

            state.m_cubeSize[iCube] = 1.0f;
        }
        // Set camera looking at cube.
        {
            state.m_camInWorldX = 0.0f;
            state.m_camInWorldY = -4.0f;
            state.m_camInWorldZ = 0.0f;

            const Quatf q = FromAngleAxis(kHalfPi, Vec3f{-1.0f, 0.0f, 0.0f});
            state.m_camInWorldW = q[0];
            state.m_camInWorldE23 = q[1];
            state.m_camInWorldE13 = q[2];
            state.m_camInWorldE12 = q[3];
        }

        constexpr WNDCLASSEX kWindowClass = {
            .cbSize = sizeof(WNDCLASSEX),
            .style = CS_HREDRAW | CS_VREDRAW,
            .lpfnWndProc = ProcessCallback,
            .lpszClassName = "engine",
        };
        RegisterClassEx(&kWindowClass);

        resources.m_hWindow = CreateWindowEx(
            0,
            kWindowClass.lpszClassName,
            "Engine",
            WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_VISIBLE,
            CW_USEDEFAULT, CW_USEDEFAULT, kWindowWidth, kWindowHeight,
            nullptr,
            nullptr,
            GetModuleHandle(nullptr),
            &resources
            );

        const RAWINPUTDEVICE rid[] = {
            {
                .usUsagePage = 0x01,
                .usUsage = 0x02,
                .dwFlags = RIDEV_INPUTSINK,
                .hwndTarget = resources.m_hWindow,
            },
            {
                .usUsagePage = 0x01,
                .usUsage = 0x06,
                .dwFlags = RIDEV_INPUTSINK,
                .hwndTarget = resources.m_hWindow,
            }
        };
        RegisterRawInputDevices(rid, 2, sizeof(RAWINPUTDEVICE));

        ShowWindow(resources.m_hWindow, SW_SHOW);
        UpdateWindow(resources.m_hWindow);

        while (state.m_isRunning)
        {
            resources.m_mouseX = 0;
            resources.m_mouseY = 0;
            resources.m_forward = false;
            resources.m_backward = false;
            resources.m_left = false;
            resources.m_right = false;

            MSG msg;
            while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
            {
                if (msg.message == WM_QUIT)
                {
                    state.m_isRunning = false;
                }

                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }

            HandleInput(resources, state);

            RenderFrame(resources, state);

            Sleep(1);
        }
    }
}

int main()
{
    using namespace Engine;

    Resources *pResources = new Resources();
    State *pState = new State();

    Run(*pResources, *pState);

    delete pResources;
    delete pState;

    return 0;
}
