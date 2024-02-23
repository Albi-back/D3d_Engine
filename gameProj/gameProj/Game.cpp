//
// Game.cpp
//

#include "pch.h"
#include "Game.h"
#include <SpriteBatch.h>

extern void ExitGame() noexcept;

using namespace DirectX;
using namespace DirectX::SimpleMath;
using namespace std;

using Microsoft::WRL::ComPtr;
namespace {
    const XMVECTORF32 START_POSITION = { 0.f,-1.5f,0.f,0.f };
    const XMVECTORF32 ROOM_BOUNDS = { 8.f, 6.f, 12.f, 0.f };
    constexpr float ROTATION_GAIN = 0.004f;
    constexpr float MOVEMENT_GAIN = 0.07f;
}

Game::Game() noexcept(false):
    m_pitch(0),
    m_yaw(0),
    m_cameraPos(START_POSITION),
    m_roomColor(Colors::White)
{
    m_deviceResources = std::make_unique<DX::DeviceResources>();
    // TODO: Provide parameters for swapchain format, depth/stencil format, and backbuffer count.
    //   Add DX::DeviceResources::c_AllowTearing to opt-in to variable rate displays.
    //   Add DX::DeviceResources::c_EnableHDR for HDR10 display.
    m_deviceResources->RegisterDeviceNotify(this);
}

// Initialize the Direct3D resources required to run.
void Game::Initialize(HWND window, int width, int height)
{
    m_deviceResources->SetWindow(window, width, height);

    m_deviceResources->CreateDeviceResources();
    CreateDeviceDependentResources();

    m_deviceResources->CreateWindowSizeDependentResources();
    CreateWindowSizeDependentResources();
    m_keyboard = make_unique<Keyboard>();
    m_mouse = make_unique<Mouse>();
    m_mouse->SetWindow(window);
    // TODO: Change the timer settings if you want something other than the default variable timestep mode.
    // e.g. for 60 FPS fixed timestep update logic, call:
    /*
    m_timer.SetFixedTimeStep(true);
    m_timer.SetTargetElapsedSeconds(1.0 / 60);
    */
}

#pragma region Frame Update
// Executes the basic game loop.
void Game::Tick()
{
    m_timer.Tick([&]()
    {
        Update(m_timer);
    });

    Render();
}

// Updates the world.
void Game::Update(DX::StepTimer const& timer)
{
    float elapsedTime = float(timer.GetElapsedSeconds());

    // TODO: Add your game logic here.
    auto time = static_cast<float>(timer.GetTotalSeconds());
    Vector3 pos = Vector3::Lerp(Vector3::Zero, Vector3::One, cos(time));
    m_world = Matrix::CreateRotationY(time)* Matrix::CreateTranslation(pos);
    auto kb = m_keyboard->GetState();
    auto mouse = m_mouse->GetState();
    if (mouse.positionMode == Mouse::MODE_RELATIVE)
    {
        Vector3 delta = Vector3(float(mouse.x), float(mouse.y), 0.f)
            * ROTATION_GAIN;

        m_pitch -= delta.y;
        m_yaw -= delta.x;
    }

    m_mouse->SetMode(mouse.leftButton
        ? Mouse::MODE_RELATIVE : Mouse::MODE_ABSOLUTE);
    if (kb.Escape)
    {
        ExitGame();
    }
    
    constexpr float limit = XM_PIDIV2 - 0.01f;
    m_pitch = max(-limit, m_pitch);
    m_pitch = min(+limit, m_pitch);

    if (m_yaw > XM_PI)
    {
        m_yaw -= XM_2PI;
    }
    else if (m_yaw < -XM_PI)
    {
        m_yaw += XM_2PI;
    }

    float y = sinf(m_pitch);
    float r = cosf(m_pitch);
    float z = r * cosf(m_yaw);
    float x = r * sinf(m_yaw);
    XMVECTOR lookAt = m_cameraPos + Vector3(x, y, z);
    m_view = XMMatrixLookAtRH(m_cameraPos, lookAt, Vector3::Up);
    elapsedTime;
    if (kb.Escape)
    {
        ExitGame();
    }

    if (kb.Home)
    {
        m_cameraPos = START_POSITION.v;
        m_pitch = m_yaw = 0;
    }

    Vector3 move = Vector3::Zero;

    if (kb.Up || kb.Space)
        move.y += 1.f;

    if (kb.Down || kb.X)
        move.y -= 1.f;

    if (kb.Left || kb.A)
        move.x += 1.f;

    if (kb.Right || kb.D)
        move.x -= 1.f;

    if (kb.PageUp || kb.W)
        move.z += 1.f;

    if (kb.PageDown || kb.S)
        move.z -= 1.f;

    Quaternion q = Quaternion::CreateFromYawPitchRoll(m_yaw, m_pitch, 0.f);

    move = Vector3::Transform(move, q);

    move *= MOVEMENT_GAIN;

    m_cameraPos += move;

    Vector3 halfBound = (Vector3(ROOM_BOUNDS.v) / Vector3(2.f))
        - Vector3(0.1f, 0.1f, 0.1f);

    m_cameraPos = Vector3::Min(m_cameraPos, halfBound);
    m_cameraPos = Vector3::Max(m_cameraPos, -halfBound);

}
#pragma endregion

#pragma region Frame Render
// Draws the scene.
void Game::Render()
{
    // Don't try to render anything before the first Update.
    if (m_timer.GetFrameCount() == 0)
    {
        return;
    }

    Clear();

    m_deviceResources->PIXBeginEvent(L"Render");
    auto context = m_deviceResources->GetD3DDeviceContext();
    float time = float(m_timer.GetTotalSeconds());
    // TODO: Add your rendering code here.
    
    
    context->OMSetBlendState(m_states->Opaque(), nullptr, 0xFFFFFFFF);
    context->OMSetDepthStencilState(m_states->DepthNone(), 0);
    context->RSSetState(m_states->CullNone());

    m_effect->Apply(context);

    context->IASetInputLayout(m_inputLayout.Get());

    m_batch->Begin();

    //VertexPositionColor v1(Vector3(0.f, 0.5f, 0.5f), Colors::Yellow);
    //VertexPositionColor v2(Vector3(0.5f, -0.5f, 0.5f), Colors::Yellow);
    //VertexPositionColor v3(Vector3(-0.5f, -0.5f, 0.5f), Colors::Yellow);

   // m_batch->DrawTriangle(v1, v2, v3);
    //m_shape->Draw(m_world, m_view, m_proj,Colors::White, m_texture.Get());
    //m_model->Draw(context, *m_states, m_world, m_view, m_proj);
    //m_effect->SetWorld(m_world);
    //m_shape->Draw(m_effect.get(), m_inputLayout.Get());
    m_room->Draw(Matrix::Identity, m_view, m_proj,
        m_roomColor, m_roomTex.Get());
    m_batch->End();
    context;

    m_deviceResources->PIXEndEvent();

    // Show the new frame.
    m_deviceResources->Present();
    m_graphicsMemory->Commit();
}

// Helper method to clear the back buffers.
void Game::Clear()
{
    m_deviceResources->PIXBeginEvent(L"Clear");

    // Clear the views.
    auto context = m_deviceResources->GetD3DDeviceContext();
    auto renderTarget = m_deviceResources->GetRenderTargetView();
    auto depthStencil = m_deviceResources->GetDepthStencilView();

    context->ClearRenderTargetView(renderTarget, Colors::CornflowerBlue);
    context->ClearDepthStencilView(depthStencil, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
    context->OMSetRenderTargets(1, &renderTarget, depthStencil);

    // Set the viewport.
    auto const viewport = m_deviceResources->GetScreenViewport();
    context->RSSetViewports(1, &viewport);

    m_deviceResources->PIXEndEvent();
}
#pragma endregion

#pragma region Message Handlers
// Message handlers
void Game::OnActivated()
{
    // TODO: Game is becoming active window.
}

void Game::OnDeactivated()
{
    // TODO: Game is becoming background window.
}

void Game::OnSuspending()
{
    // TODO: Game is being power-suspended (or minimized).
}

void Game::OnResuming()
{
    m_timer.ResetElapsedTime();

    // TODO: Game is being power-resumed (or returning from minimize).
}

void Game::OnWindowMoved()
{
    auto const r = m_deviceResources->GetOutputSize();
    m_deviceResources->WindowSizeChanged(r.right, r.bottom);
}

void Game::OnDisplayChange()
{
    m_deviceResources->UpdateColorSpace();
}

void Game::OnWindowSizeChanged(int width, int height)
{
    if (!m_deviceResources->WindowSizeChanged(width, height))
        return;

    CreateWindowSizeDependentResources();

    // TODO: Game window is being resized.
}

// Properties
void Game::GetDefaultSize(int& width, int& height) const noexcept
{
    // TODO: Change to desired default window size (note minimum size is 320x200).
    width = 800;
    height = 600;
}
#pragma endregion

#pragma region Direct3D Resources
// These are the resources that depend on the device.
void Game::CreateDeviceDependentResources()
{
    auto device = m_deviceResources->GetD3DDevice();
    m_graphicsMemory = make_unique<GraphicsMemory>(device);
    m_states = make_unique<CommonStates>(device);
    m_effect = make_unique<BasicEffect>(device);
    m_effect->SetTextureEnabled(true);
    m_effect->SetPerPixelLighting(true);
    m_effect->SetLightingEnabled(true);
    m_effect->SetLightEnabled(0, true);
    m_effect->SetLightDiffuseColor(0, Colors::White);
    m_effect->SetLightDirection(0, -Vector3::UnitZ);
    //m_effect->SetVertexColorEnabled(true);
 
    auto context = m_deviceResources->GetD3DDeviceContext();
    m_batch = make_unique<PrimitiveBatch<VertexType>>(context);

    // TODO: Initialize device dependent objects here (independent of window size).
    m_shape = GeometricPrimitive::CreateSphere(context);
    DX::ThrowIfFailed(
        CreateWICTextureFromFile(device, L"earth.bmp",nullptr,
            m_texture.ReleaseAndGetAddressOf())
    );
    m_states = make_unique<CommonStates>(device);
    m_fxFactory = make_unique<EffectFactory>(device);
    m_model = Model::CreateFromCMO(device, L"cup.cmo", *m_fxFactory);
    m_room = GeometricPrimitive::CreateBox(context,
        XMFLOAT3(ROOM_BOUNDS[0], ROOM_BOUNDS[1], ROOM_BOUNDS[2]),
        false, true);
    DX::ThrowIfFailed(
        CreateDDSTextureFromFile(device, L"roomtexture.dds",
            nullptr, m_roomTex.ReleaseAndGetAddressOf()));
    m_world = Matrix::Identity;
   

    device;
}

// Allocate all memory resources that change on a window SizeChanged event.
void Game::CreateWindowSizeDependentResources()
{
    // TODO: Initialize windows-size dependent objects here.
    auto size = m_deviceResources->GetOutputSize();
    m_view = Matrix::CreateLookAt(Vector3(2.f, 2.f, 2.f),
        Vector3::Zero, Vector3::UnitY);
    m_proj = Matrix::CreatePerspectiveFieldOfView(
        XMConvertToRadians(70.f),
        float(size.right) / float(size.bottom), 0.01f, 100.f);
    m_effect->SetView(m_view);
    m_effect->SetProjection(m_proj);

}

void Game::OnDeviceLost()
{
    
    // TODO: Add Direct3D resource cleanup here.
    m_states.reset();
    m_effect.reset();
    m_batch.reset();
    m_inputLayout.Reset();
    m_shape.reset();
    m_texture.Reset();
    m_room.reset();
    m_roomTex.Reset();
}

void Game::OnDeviceRestored()
{
    CreateDeviceDependentResources();

    CreateWindowSizeDependentResources();
}
#pragma endregion
