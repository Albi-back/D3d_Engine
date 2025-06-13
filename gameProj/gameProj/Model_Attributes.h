#pragma once

#include <DirectXMath.h>
#include <wrl/client.h>
#include <d3d11.h>
#include <functional> // for std::hash

struct Vertex
{
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT3 normal;
    DirectX::XMFLOAT2 texcoord;

    bool operator==(const Vertex& other) const
    {
        return position.x == other.position.x && position.y == other.position.y && position.z == other.position.z &&
            normal.x == other.normal.x && normal.y == other.normal.y && normal.z == other.normal.z &&
            texcoord.x == other.texcoord.x && texcoord.y == other.texcoord.y;
    }
};

struct VertexHasher
{
    size_t operator()(const Vertex& v) const
    {
        size_t h1 = std::hash<float>{}(v.position.x);
        size_t h2 = std::hash<float>{}(v.position.y);
        size_t h3 = std::hash<float>{}(v.position.z);
        size_t h4 = std::hash<float>{}(v.normal.x);
        size_t h5 = std::hash<float>{}(v.normal.y);
        size_t h6 = std::hash<float>{}(v.normal.z);
        size_t h7 = std::hash<float>{}(v.texcoord.x);
        size_t h8 = std::hash<float>{}(v.texcoord.y);

        // Combine hashes (Boost-like hash combine)
        size_t seed = h1;
        seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h4 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h5 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h6 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h7 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h8 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct MyModel
{
    UINT m_vertexCount = 0;
    UINT m_indexCount = 0;

    Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexBuffer;
    Microsoft::WRL::ComPtr<ID3D11Buffer> m_indexBuffer;

    // Optional extra stuff like input layout, materials, etc.
    Microsoft::WRL::ComPtr<ID3D11InputLayout> m_inputLayout;
};
