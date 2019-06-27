package com.gemsrobotics.util;

import org.usfirst.frc.team3310.utility.math.Interpolable;

import java.awt.*;
import java.text.DecimalFormat;

import static java.lang.Math.*;

// thank you d3 https://github.com/d3/d3-color
public final class ColorInterpolator {
	private double checkLerpVal(final double val) {
		if (val < 0) {
			return 0;
		} else if (val > 1) {
			return 1;
		} else {
			return val;
		}
	}

	public Color lerp(final Color c1, final Color c2, final double n) {
		return HCL.fromRGB(c1).interpolate(HCL.fromRGB(c2), checkLerpVal(n)).toRGB();
	}

	private static class HCL implements Interpolable<HCL> {
		private static final double
				t0 = 4 / 29.0,
				t1 = 6 / 29.0,
				t2 = 3 * t1 * t1,
				t3 = t1 * t1 * t1;

		private static final double
				Xn = 0.96422,
				Yn = 1,
				Zn = 0.82521;

		public final double h, c, l, o;

		public HCL(final double h, final double c, final double l, final double o) {
			this.h = h;
			this.c = c;
			this.l = l;
			this.o = o;
		}

		private static double rgb2lrgb(double n) {
			n /= 255.0;
			return n < 0.04045 ? n / 12.92 : pow((n + 0.055) / 1.055, 2.4);
		}

		private static double lrgb2rgb(double n) {
			return 255 * (n <= 0.0031308 ? 12.92 * n : 1.055 * pow(n, 1 / 2.4) - 0.055);
		}

		private static double xyz2lab(final double n) {
			return n > t3 ? pow(n, 0.3333333) : n / t2 + t0;
		}

		private static double lab2xyz(final double n) {
			return n > t1 ? n * n * n : t2 * (n - t0);
		}

		public static HCL fromRGB(final Color color) {
			final double r = rgb2lrgb(color.getRed()),
					g = rgb2lrgb(color.getGreen()),
					b = rgb2lrgb(color.getBlue()),
					y = xyz2lab((0.2225045 * r + 0.7168786 * g + 0.0606169 * b) / Yn);

			final double x, z;

			if (r == g && g == b) {
				x = y;
				z = y;
			} else {
				x = xyz2lab((0.4360747 * r + 0.3850649 * g + 0.1430804 * b) / Xn);
				z = xyz2lab((0.0139322 * r + 0.0971045 * g + 0.7141733 * b) / Zn);
			}

			final double
					l = 116 * y - 16,
					a = 500 * x - y,
					b1 = 200 * (y - z),
					o = color.getAlpha();

			final double h = toDegrees(atan2(b1, a));

			return new HCL(
					h < 0 ? h + 360 : h,
					sqrt(a * a + b1 * b1),
					l,
					o
			);
		}

		public Color toRGB() {
			final double rH = toRadians(h);
			final double
					newL = l,
					a = cos(rH) * c,
					b = sin(rH) * c;

			double
					y = (newL + 16) / 116.0,
					x = Double.isNaN(a) ? y : y + a / 500,
					z = Double.isNaN(b) ? y : y - b / 200;

			x = Xn * lab2xyz(x);
			y = Yn * lab2xyz(y);
			z = Zn * lab2xyz(z);

			return new Color(
					(float) lrgb2rgb( 3.1338561 * x - 1.6168667 * y - 0.4906146 * z),
					(float) lrgb2rgb(-0.9787684 * x + 1.9161415 * y + 0.0334540 * z),
					(float) lrgb2rgb( 0.0719453 * x - 0.2289914 * y + 1.4052427 * z),
					(float) o
			);
		}

		private static final DecimalFormat dcf = new DecimalFormat("#0.000");

		public HCL interpolate(final HCL other, final double n) {
			return new HCL(
					(other.h - h) * n + h,
					(other.c - c) * n + c,
					(other.l - l) * n + l,
					(other.o - o) * n + o
			);
		}

		@Override
		public String toString() {
			return String.format(
					"HCL[h: %s, c: %s, l: %s, o: %s]",
					dcf.format(h),
					dcf.format(c),
					dcf.format(l),
					dcf.format(o));
		}
	}
}
