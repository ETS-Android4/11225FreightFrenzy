package org.firstinspires.ftc.teamcode.autonomous.enums;

public enum Position {
    RIGHT("right"),
    LEFT("left")
    ;

    private final String text;

    Position(final String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return text;
    }
}
