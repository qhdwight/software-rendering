#pragma once

#include <cstdint>

namespace Engine
{
    using U8 = std::uint8_t;
    using U16 = std::uint16_t;
    using U32 = std::uint32_t;
    using U64 = std::uint64_t;
    using U128 = __uint128_t;

    using I8 = std::int8_t;
    using I16 = std::int16_t;
    using I32 = std::int32_t;
    using I64 = std::int64_t;
    using I128 = __int128_t;

    using F32 = float;

    typedef float F32x2 __attribute__((__vector_size__(8), __aligned__(8)));
    typedef float F32x4 __attribute__((__vector_size__(16), __aligned__(16)));

    static constexpr F32 kPi = 3.1415927f;
    static constexpr F32 kTau = 6.2831853f;
    static constexpr F32 kHalfPi = 1.5707963f;
    static constexpr F32 kInvTau = 0.15915494f;

    constexpr F32 Sin(F32 x)
    {
        // TODO: More rigorous and accurate.
        // Range reduction to [-pi, pi]
        x -= kTau * static_cast<F32>(static_cast<I32>(x * kInvTau));
        // Further range reduction to [-pi/2, pi/2] and exploit symmetry.
        x = x > kHalfPi ? kPi - x : x; // Sin(x) = Sin(pi - x).
        x = x < -kHalfPi ? -kPi - x : x; // Sin(x) = Sin(-pi - x).
        // Taylor series approximation.
        const F32 x2 = x * x;
        return x * (1.0f - x2 * (1.0f / 6.0f - x2 * (1.0f / 120.0f - x2 * (1.0f / 5040.0f))));
    }

    constexpr F32 Cos(const F32 x)
    {
        return Sin(x + kHalfPi);
    }

    constexpr F32 Tan(const F32 angle)
    {
        return Sin(angle) / Cos(angle);
    }

    inline F32 Sqrt(const F32 value)
    {
        return __builtin_sqrtf(value);
    }

    constexpr F32 Abs(const F32 value)
    {
        return __builtin_fabsf(value);
    }

    constexpr F32 Min(const F32 a, const F32 b)
    {
        return a < b ? a : b;
    }

    constexpr F32 Max(const F32 a, const F32 b)
    {
        return a > b ? a : b;
    }

    template<U32 k1, U32 k2, U32 k3, U32 k4>
    constexpr F32x4 Shuffle(const F32x4 v)
    {
        return __builtin_shufflevector(v, v, k1, k2, k3, k4);
    }

    struct Vec2f
    {
        F32x2 m_v;

        constexpr explicit Vec2f(const F32x2 v) :
            m_v{v}
        {
        }

        constexpr Vec2f(const float x, const float y) :
            m_v{x, y}
        {
        }

        constexpr float operator[](const U32 index) const
        {
            return m_v[index];
        }
    };

    struct Vec3f
    {
        F32x4 m_v;

        constexpr explicit Vec3f(const F32x4 v) :
            m_v{v}
        {
        }

        constexpr Vec3f(const float x, const float y, const float z) :
            m_v{x, y, z, 0.0f}
        {
        }

        constexpr float operator[](const U32 index) const
        {
            return m_v[index];
        }

        constexpr friend Vec3f operator*(const float scalar, const Vec3f v)
        {
            return Vec3f(scalar * v.m_v);
        }

        constexpr friend Vec3f operator*(const Vec3f v, const float scalar)
        {
            return Vec3f{v.m_v * scalar};
        }

        constexpr friend Vec3f operator/(const Vec3f v, const float scalar)
        {
            return Vec3f{v.m_v / scalar};
        }

        constexpr friend Vec3f operator+(const Vec3f a, const Vec3f b)
        {
            return Vec3f{a.m_v + b.m_v};
        }

        constexpr friend Vec3f operator-(const Vec3f a, const Vec3f b)
        {
            return Vec3f{a.m_v - b.m_v};
        }

        constexpr friend Vec3f operator*(const Vec3f a, const Vec3f b)
        {
            return Vec3f{a.m_v * b.m_v};
        }

        constexpr friend Vec3f operator-(const Vec3f v)
        {
            return Vec3f{-v.m_v};
        }
    };

    constexpr Vec3f Cross(const Vec3f a, const Vec3f b)
    {
        const F32x4 d1 = Shuffle<1, 2, 0, 3>(a.m_v) * Shuffle<2, 0, 1, 3>(b.m_v);
        const F32x4 d2 = Shuffle<2, 0, 1, 3>(a.m_v) * Shuffle<1, 2, 0, 3>(b.m_v);
        return Vec3f(d1 - d2);
    }

    constexpr F32 Dot(const F32x4 a, const F32x4 b)
    {
        const F32x4 m0 = a * b;
        const F32x4 m1 = Shuffle<1, 0, 3, 2>(m0) + m0;
        const F32x4 m2 = Shuffle<2, 3, 0, 1>(m1) + m1;
        return m2[0];
    }

    inline Vec3f Normalize(const Vec3f v)
    {
        return v / Sqrt(Dot(v.m_v, v.m_v));
    }

    struct Quatf
    {
        F32x4 m_v;

        constexpr explicit Quatf(const F32x4 v) :
            m_v{v}
        {
        }

        constexpr Quatf(const float w, const float e23, const float e13, const float e12) :
            m_v{w, e23, e13, e12}
        {
        }

        constexpr float operator[](const U32 index) const
        {
            return m_v[index];
        }

        constexpr friend Quatf operator*(const Quatf a, const Quatf b);
    };

    constexpr Quatf Conjugate(const Quatf a)
    {
        // Invert the bivector.
        return Quatf{a.m_v * F32x4{1.0f, -1.0f, -1.0f, -1.0f}};
    }

    constexpr Quatf operator*(const Quatf a, const Quatf b)
    {
        F32x4 a0 = Shuffle<0, 0, 0, 0>(a.m_v);
        F32x4 a1 = Shuffle<1, 1, 1, 1>(a.m_v);
        F32x4 a2 = Shuffle<2, 2, 2, 2>(a.m_v);
        F32x4 a3 = Shuffle<3, 3, 3, 3>(a.m_v);
        F32x4 b0 = Shuffle<0, 1, 2, 3>(b.m_v);
        F32x4 b1 = Shuffle<1, 0, 3, 2>(b.m_v);
        F32x4 b2 = Shuffle<2, 3, 0, 1>(b.m_v);
        F32x4 b3 = Shuffle<3, 2, 1, 0>(b.m_v);
        a1[0] *= -1.0f;
        a1[2] *= -1.0f;
        a2[0] *= -1.0f;
        a2[3] *= -1.0f;
        a3[0] *= -1.0f;
        a3[1] *= -1.0f;
        return Quatf{a0 * b0 + (a2 * b2 + (a1 * b1 + a3 * b3))};
    }

    constexpr Vec3f Rotate(const Quatf q, const Vec3f v)
    {
        const Vec3f u(Shuffle<1, 2, 3, 0>(q.m_v));
        const Vec3f uv = Cross(u, v);
        return v + 2.0f * (Cross(u, uv) + q[0] * uv);
    }

    constexpr Vec3f InverseRotate(const Quatf q, const Vec3f v)
    {
        const Vec3f u(Shuffle<1, 2, 3, 0>(q.m_v));
        const Vec3f uv = Cross(u, v);
        return v + 2.0f * (q[0] * uv - Cross(u, uv));
    }

    inline Quatf FromAngleAxis(const F32 angle, const Vec3f axis)
    {
        const F32 cosHalfAngle = Cos(angle * 0.5f);
        //    Sin^2(theta/2) + Cos^2(theta/2) = 1
        // => Sin(theta/2) = Sqrt(1 - Cos^2(theta/2))
        const F32 sinHalfAngle = Sqrt(1.0f - cosHalfAngle * cosHalfAngle) * (angle >= 0.0f ? 1.0f : -1.0f);
        const Vec3f scaledAxis = Normalize(axis) * sinHalfAngle;
        return Quatf{cosHalfAngle, scaledAxis[0], scaledAxis[1], scaledAxis[2]};
    }

    inline Quatf Normalize(const Quatf q)
    {
        return Quatf(q.m_v / Sqrt(Dot(q.m_v, q.m_v)));
    }

    struct Pose
    {
        Vec3f m_pos;
        Quatf m_ori;

        constexpr Pose(const Vec3f pos, const Quatf ori) :
            m_pos{pos}, m_ori{ori}
        {
        }
    };

    constexpr Pose Transform(const Pose &a, const Pose &b)
    {
        const Quatf ori = a.m_ori * b.m_ori;
        const Vec3f pos = a.m_pos + Rotate(a.m_ori, b.m_pos);
        return Pose{pos, ori};
    }

    constexpr Pose Inverse(const Pose &pose)
    {
        //    T(x) = R @ x + t
        // => R^-1 @ T(x) = x + R^-1 @ t
        // => x = R^-1 @ T(x) - R^-1 @ t
        // Thus T^-1(x) = R^-1 @ x - R^-1 @ t

        // Assume the quaternion is unitized so that the conjugate is the inverse.
        const Quatf ori = Conjugate(pose.m_ori);
        // Apply the inverse rotation since it is cheap for quaternions.
        const Vec3f pos = -InverseRotate(pose.m_ori, pose.m_pos);
        return Pose{pos, ori};
    }

    constexpr Vec3f Transform(const Pose &pose, const Vec3f &v)
    {
        return pose.m_pos + Rotate(pose.m_ori, v);
    }
}
