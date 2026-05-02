#ifndef VECTOR2D_H
#define VECTOR2D_H

class Vector2D {
public:
    float x, y;

    Vector2D() : x(0), y(0) {}
    Vector2D(float _x, float _y) : x(_x), y(_y) {}

    float magnitude() const;
    Vector2D normalized() const;

    Vector2D operator+(const Vector2D& other) const;
    Vector2D operator-(const Vector2D& other) const;
    Vector2D operator*(float scalar) const;
    Vector2D operator/(float scalar) const;

    Vector2D& operator+=(const Vector2D& other);
    Vector2D& operator-=(const Vector2D& other);
    Vector2D& operator*=(float scalar);
    Vector2D& operator/=(float scalar);

    bool operator==(const Vector2D& other) const;
    bool operator!=(const Vector2D& other) const;
};

float Vector2D::magnitude() const {
    return sqrt(x * x + y * y);
}

Vector2D Vector2D::normalized() const {
    float mag = magnitude();
    if (mag != 0) {
        return Vector2D(x / mag, y / mag);
    }
    return Vector2D(0, 0);
}

Vector2D Vector2D::operator+(const Vector2D& other) const {
    return Vector2D(x + other.x, y + other.y);
}

Vector2D Vector2D::operator-(const Vector2D& other) const {
    return Vector2D(x - other.x, y - other.y);
}

Vector2D Vector2D::operator*(float scalar) const {
    return Vector2D(x * scalar, y * scalar);
}

Vector2D Vector2D::operator/(float scalar) const {
    return Vector2D(x / scalar, y / scalar);
}

Vector2D& Vector2D::operator+=(const Vector2D& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Vector2D& Vector2D::operator-=(const Vector2D& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Vector2D& Vector2D::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

Vector2D& Vector2D::operator/=(float scalar) {
    x /= scalar;
    y /= scalar;
    return *this;
}

bool Vector2D::operator==(const Vector2D& other) const {
    return x == other.x && y == other.y;
}

bool Vector2D::operator!=(const Vector2D& other) const {
    return !(*this == other);
}

#endif // VECTOR2D_H
