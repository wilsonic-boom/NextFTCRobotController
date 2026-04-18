package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class ShooterCalc {
    private static final String[] COLUMNS = {"Distance", "Velocity", "HoodServoPos"};

    private static final double[][] TABLE = {
            // Distance Velocity HoodServoPos
            {93, 1460, 0.19},
            {76, 1360, 0.17},
            {59, 1200, 0.12},
            {25, 1080, 0},
            {142, 1640, 0.17},
    };

    private final Map<String, Integer> colIndex = new HashMap<>();
    private final Map<String, double[][]> splineCache = new HashMap<>();

    private final double[] xs;

    private static double distanceMultiplier = 1.0;
    private static final double MULTIPLIER_STEP = 0.02;

    public static void adjustDistance(String direction) {
        if (direction.equalsIgnoreCase("tooClose")) {
            distanceMultiplier += MULTIPLIER_STEP;
        } else if (direction.equalsIgnoreCase("tooFar")) {
            distanceMultiplier -= MULTIPLIER_STEP;
        }
    }

    public static void resetDistanceMultiplier() {
        distanceMultiplier = 1.0;
    }


    public ShooterCalc() {
        for (int i = 0; i < COLUMNS.length; i++) {
            colIndex.put(COLUMNS[i].toLowerCase(), i);
        }

        Arrays.sort(TABLE, (a, b) -> Double.compare(a[0], b[0]));

        xs = new double[TABLE.length];
        for (int i = 0; i < TABLE.length; i++) xs[i] = TABLE[i][0];
    }

    public double lookup(double xQuery, String column) {
        String key = column.toLowerCase();
        if (!colIndex.containsKey(key)) {
            throw new IllegalArgumentException("Unknown column: " + column
                    + ". Available: " + colIndex.keySet());
        }
        double[][] coeffs = splineCache.computeIfAbsent(key, this::buildSpline);
        double result = evaluate(coeffs, xQuery * distanceMultiplier);
        if (key.equals("hoodservopos")) {
            return Math.min(1.0, Math.max(0.0, result));
        } else {
            return Math.max(0.0, result);
        }
    }

    // some online algoithim or something
    private double[][] buildSpline(String colKey) {
        int colIdx = colIndex.get(colKey);
        int n = TABLE.length;
        double[] y = new double[n];
        for (int i = 0; i < n; i++) y[i] = TABLE[i][colIdx];

        int m = n - 1;
        double[] h = new double[m];
        for (int i = 0; i < m; i++) h[i] = xs[i + 1] - xs[i];

        double[] alpha = new double[n];
        for (int i = 1; i < m; i++) {
            alpha[i] = (3.0 / h[i]) * (y[i + 1] - y[i])
                    - (3.0 / h[i - 1]) * (y[i] - y[i - 1]);
        }

        double[] l = new double[n];
        double[] mu = new double[n];
        double[] z = new double[n];
        l[0] = 1.0;

        for (int i = 1; i < m; i++) {
            l[i]  = 2.0 * (xs[i + 1] - xs[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i]  = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        l[m] = 1.0;

        double[] c = new double[n];
        for (int j = m - 1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
        }

        double[][] coeffs = new double[m][4];
        for (int i = 0; i < m; i++) {
            double a = y[i];
            double b = (y[i + 1] - y[i]) / h[i]
                    - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
            double d = (c[i + 1] - c[i]) / (3.0 * h[i]);
            coeffs[i] = new double[]{a, b, c[i], d};
        }
        return coeffs;
    }

    private double evaluate(double[][] coeffs, double xQuery) {
        int n = coeffs.length;
        int seg = n - 1;
        for (int i = 0; i < n - 1; i++) {
            if (xQuery <= xs[i + 1]) { seg = i; break; }
        }
        if (xQuery < xs[0]) seg = 0;

        double dx = xQuery - xs[seg];
        double[] c = coeffs[seg];
        return c[0] + dx * (c[1] + dx * (c[2] + dx * c[3]));
    }

    public static void main(String[] args) {
        ShooterCalc si = new ShooterCalc ();

        double[] testX = {25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 130, 140, 150};

        System.out.printf("%-10s  %-12s  %-15s%n", "Distance", "Velocity", "HoodServoPos");
        System.out.println("-".repeat(42));
        for (double x : testX) {
            System.out.printf("%-10.2f  %-12.6f  %-15.6f%n",
                    x, si.lookup(x, "Velocity"), si.lookup(x, "HoodServoPos"));
        }
    }
}