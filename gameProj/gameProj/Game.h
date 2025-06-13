//
// Game.h
//

#pragma once

#include "DeviceResources.h"
#include "StepTimer.h"
#include "Model_Attributes.h"
#include <unordered_map>
using namespace std;

// A basic game implementation that creates a D3D11 device and
// provides a game loop.
class Game final : public DX::IDeviceNotify
{
public:

    Game() noexcept(false);
    ~Game() = default;

    Game(Game&&) = default;
    Game& operator= (Game&&) = default;

    Game(Game const&) = delete;
    Game& operator= (Game const&) = delete;

    // Initialization and management
    void Initialize(HWND window, int width, int height);

    // Basic game loop
    void Tick();
    void LoadModel(const std::string& filename, ID3D11Device* device);

    // IDeviceNotify
    void OnDeviceLost() override;
    void OnDeviceRestored() override;

    // Messages
    void OnActivated();
    void OnDeactivated();
    void OnSuspending();
    void OnResuming();
    void OnWindowMoved();
    void OnDisplayChange();
    void OnWindowSizeChanged(int width, int height);
    std::unordered_map<std::string, MyModel> m_Models;
    // Properties
    void GetDefaultSize( int& width, int& height ) const noexcept;
    
private:

    void Update(DX::StepTimer const& timer);
    void Render();

    void Clear();

    void CreateDeviceDependentResources();
    void CreateWindowSizeDependentResources();

    // Device resources.
    std::unique_ptr<DX::DeviceResources>    m_deviceResources;

    // Rendering loop timer.
    DX::StepTimer                           m_timer;
    
    std::unique_ptr<DirectX::GraphicsMemory> m_graphicsMemory;

    using VertexType = DirectX::VertexPositionColor;
    unique_ptr<DirectX::CommonStates> m_states;
    unique_ptr<DirectX::BasicEffect> m_effect;
    unique_ptr<DirectX::PrimitiveBatch<VertexType>> m_batch;
    Microsoft::WRL::ComPtr<ID3D11InputLayout> m_inputLayout;
    DirectX::SimpleMath::Matrix m_world;
    DirectX::SimpleMath::Matrix m_view;
    DirectX::SimpleMath::Matrix m_proj;

    std::unique_ptr<DirectX::GeometricPrimitive> m_shape;
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_texture;
    unique_ptr<DirectX::IEffectFactory> m_fxFactory;
    unique_ptr<DirectX::Model> m_model;
    unique_ptr<DirectX::Keyboard> m_keyboard;
    unique_ptr<DirectX::Mouse> m_mouse;
    unique_ptr<DirectX::GeometricPrimitive> m_room;
    float m_pitch;
    float m_yaw;
    DirectX::SimpleMath::Vector3 m_cameraPos;

    DirectX::SimpleMath::Color m_roomColor;
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_roomTex;
   
    //std::unordered_map<std::string, Texture> m_Texture;
};
