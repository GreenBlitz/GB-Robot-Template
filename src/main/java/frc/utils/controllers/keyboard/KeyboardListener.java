package frc.utils.controllers.keyboard;

import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;


class KeyboardListener implements NativeKeyListener {

    protected static boolean ESC, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10,
    F11, F12, DELETE, TILDA, ONE, TWO, THREE, FOUR,
    FIVE, SIX, SEVEN, EIGHT, NINE, ZERO, MINUS, EQUALS,
    BACKSPACE, TAB, Q, W, E, R, T, Y, U, I, O, P, A, S,
    D, F, G, H, J, K, L, SEMICOLON, APOSTROPHE, LEFT_SHIFT,
    Z, X, C, V, B, N, M, COMMA, PERIOD,
    RIGHT_SHIFT, CONTROL, LEFT_ALT, RIGHT_CONTROL,
    LEFT_ARROW, RIGHT_ARROW, UP_ARROW, DOWN_ARROW, NUMPAD_0, NUMPAD_1, NUMPAD_2,
    NUMPAD_3, NUMPAD_4, NUMPAD_5, NUMPAD_6, NUMPAD_7, NUMPAD_8,
    NUMPAD_9;

    @Override
    public void nativeKeyPressed(NativeKeyEvent nativeEvent) {
        changeBoolean(true, nativeEvent.getKeyCode());
        System.out.println(NativeKeyEvent.getKeyText(nativeEvent.getKeyCode()));
    }

    private void changeBoolean(boolean toChange, int keyCode) {
        switch (keyCode) {
            case NativeKeyEvent.VC_ESCAPE -> ESC = toChange;
            case NativeKeyEvent.VC_F1 -> F1 = toChange;
            case NativeKeyEvent.VC_F2 -> F2 = toChange;
            case NativeKeyEvent.VC_F3 -> F3 = toChange;
            case NativeKeyEvent.VC_F4 -> F4 = toChange;
            case NativeKeyEvent.VC_F5 -> F5 = toChange;
            case NativeKeyEvent.VC_F6 -> F6 = toChange;
            case NativeKeyEvent.VC_F7 -> F7 = toChange;
            case NativeKeyEvent.VC_F8 -> F8 = toChange;
            case NativeKeyEvent.VC_F9 -> F9 = toChange;
            case NativeKeyEvent.VC_F10 -> F10 = toChange;
            case NativeKeyEvent.VC_F11 -> F11 = toChange;
            case NativeKeyEvent.VC_F12 -> F12 = toChange;
            case NativeKeyEvent.VC_DELETE -> DELETE = toChange;
            case NativeKeyEvent.VC_BACKQUOTE -> TILDA = toChange;
            case NativeKeyEvent.VC_1 -> ONE = toChange;
            case NativeKeyEvent.VC_2 -> TWO = toChange;
            case NativeKeyEvent.VC_3 -> THREE = toChange;
            case NativeKeyEvent.VC_4 -> FOUR = toChange;
            case NativeKeyEvent.VC_5 -> FIVE = toChange;
            case NativeKeyEvent.VC_6 -> SIX = toChange;
            case NativeKeyEvent.VC_7 -> SEVEN = toChange;
            case NativeKeyEvent.VC_8 -> EIGHT = toChange;
            case NativeKeyEvent.VC_9 -> NINE = toChange;
            case NativeKeyEvent.VC_0 -> ZERO = toChange;
            case NativeKeyEvent.VC_MINUS -> MINUS = toChange;
            case NativeKeyEvent.VC_EQUALS -> EQUALS = toChange;
            case NativeKeyEvent.VC_BACKSPACE -> BACKSPACE = toChange;
            case NativeKeyEvent.VC_TAB -> TAB = toChange;
            case NativeKeyEvent.VC_Q -> Q = toChange;
            case NativeKeyEvent.VC_W -> W = toChange;
            case NativeKeyEvent.VC_E -> E = toChange;
            case NativeKeyEvent.VC_R -> R = toChange;
            case NativeKeyEvent.VC_T -> T = toChange;
            case NativeKeyEvent.VC_Y -> Y = toChange;
            case NativeKeyEvent.VC_U -> U = toChange;
            case NativeKeyEvent.VC_I -> I = toChange;
            case NativeKeyEvent.VC_O -> O = toChange;
            case NativeKeyEvent.VC_P -> P = toChange;
            case NativeKeyEvent.VC_A -> A = toChange;
            case NativeKeyEvent.VC_S -> S = toChange;
            case NativeKeyEvent.VC_D -> D = toChange;
            case NativeKeyEvent.VC_F -> F = toChange;
            case NativeKeyEvent.VC_G -> G = toChange;
            case NativeKeyEvent.VC_H -> H = toChange;
            case NativeKeyEvent.VC_J -> J = toChange;
            case NativeKeyEvent.VC_K -> K = toChange;
            case NativeKeyEvent.VC_L -> L = toChange;
            case NativeKeyEvent.VC_SEMICOLON -> SEMICOLON = toChange;
            case NativeKeyEvent.VC_QUOTE -> APOSTROPHE = toChange;
            case NativeKeyEvent.VC_SHIFT -> LEFT_SHIFT = toChange;
            case NativeKeyEvent.VC_Z -> Z = toChange;
            case NativeKeyEvent.VC_X -> X = toChange;
            case NativeKeyEvent.VC_C -> C = toChange;
            case NativeKeyEvent.VC_V -> V = toChange;
            case NativeKeyEvent.VC_B -> B = toChange;
            case NativeKeyEvent.VC_N -> N = toChange;
            case NativeKeyEvent.VC_M -> M = toChange;
            case NativeKeyEvent.VC_COMMA -> COMMA = toChange;
            case NativeKeyEvent.VC_PERIOD -> PERIOD = toChange;
            case 0xe36 -> RIGHT_SHIFT = toChange;
            case NativeKeyEvent.VC_CONTROL -> CONTROL = toChange;
            case NativeKeyEvent.VC_ALT -> LEFT_ALT = toChange;
        }
    }

    @Override
    public void nativeKeyReleased(NativeKeyEvent nativeEvent) {

    }
}
