package org.firstinspires.ftc.lib.math; 

public enum Unit {

    CENTIMETERS (Dimension.LENGTH, 1),
    INCHES (Dimension.LENGTH, 2.54),
    TILES (Dimension.LENGTH, 60.96);

    public Dimension dimension;
    public double toDefaultUnit;

    private Unit (Dimension d, double r) {
        dimension = d;
        toDefaultUnit = r;
    }

    public enum Dimension {
        LENGTH
    }
    
    public double to (Unit u2, double val) throws UnitMismatchException {
        if (dimension == u2.dimension) {
            return (val * toDefaultUnit) / u2.toDefaultUnit;
        }
        else {
            throw new UnitMismatchException("Inputs are of different dimensions (" + dimension.name() + "," + u2.dimension.name() + ")");
        }
    }
}