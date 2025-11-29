package org.firstinspires.ftc.teamcode.utils.math;

import androidx.annotation.NonNull;

public class Vec {
    public final double x;
    public final double y;

    public Vec(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // Add vector
    public Vec add(Vec other) {
        return new Vec(this.x + other.x, this.y + other.y);
    }

    // Subtract vector
    public Vec sub(Vec other) {
        return new Vec(this.x - other.x, this.y - other.y);
    }

    // Multiply by scalar
    public Vec mult(double s) {
        return new Vec(this.x * s, this.y * s);
    }

    // Element-wise multiply vector
    public Vec mult(Vec other) {
        return new Vec(this.x * other.x, this.y * other.y);
    }

    // Divide by scalar
    public Vec div(double s) {
        return new Vec(this.x / s, this.y / s);
    }

    // Element-wise divide vector
    public Vec div(Vec other) {
        return new Vec(this.x / other.x, this.y / other.y);
    }

    // Dot product
    public double dot(Vec other) {
        return this.x * other.x + this.y * other.y;
    }

    // Magnitude
    public double mag() {
        return Math.sqrt(x * x + y * y);
    }

    // Normalize
    public Vec normalize() {
        double m = mag();
        return (m == 0) ? new Vec(0, 0) : new Vec(x / m, y / m);
    }

    // Copy
    public Vec copy() {
        return new Vec(x, y);
    }

    @NonNull
    @Override
    public String toString() {
        return "V(" + Math.floor(x * 100)/100 + ", " + Math.floor(y * 100)/100 + ")";
    }
}
