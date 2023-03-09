package edu.wpi.first.wpilibj;

public class MyAddressableLEDBuffer extends AddressableLEDBuffer {

    public MyAddressableLEDBuffer(int length) {
        super(length);
    }

    public byte[] getBuffer() {
        return m_buffer;
    }
}
