package bbb.math;

public class bbbVector2 {
    public double x;
    public double y;

    public bbbVector2() {
        this.x = 0.0;
        this.y = 0.0;
    }

    public bbbVector2(double d) {
        this.x = d;
        this.y = d;
    }

    public bbbVector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public boolean equals(bbbVector2 other) {
        return (this.x == other.x && this.y == other.y);
    }

    public double magnitude() {
        return Math.sqrt((x * x) + (y * y));
    }

    public bbbVector2 normalize() {
        double length = Math.sqrt(x * x + y * y);

        if (length != 0.0) {
            double s = 1.0 / length;
            x = x * s;
            y = y * s;
        }

        return this;
    }

    public bbbVector2 normalize(double max) {
        double length = Math.sqrt(x * x + y * y);

        if (length != 0.0) {
            double s = max / length;
            x = x * s;
            y = y * s;
        }

        return this;
    }

    public bbbVector2 normalizeNew() {
        double length = Math.sqrt(x * x + y * y);

        bbbVector2 result = new bbbVector2();

        if (length != 0.0) {
            double s = 1.0 / length;
            result.x = x * s;
            result.y = y * s;
        }

        return result;
    }
}