use embedded_dma::WriteTarget;

#[derive(Debug, defmt::Format, Default, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct Adc1Readings {
    /// ADC123_IN0
    pub pa0: u16,
    /// ADC123_IN1
    pub pa1: u16,
    /// ADC123_IN2
    pub pa2: u16,
    /// ADC123_IN3
    pub pa3: u16,
    /// ADC12_IN4
    pub pa4: u16,
    /// ADC12_IN5
    pub pa5: u16,
    /// ADC12_IN6
    pub pa6: u16,
    /// ADC12_IN7
    pub pa7: u16,
    
}

impl Adc1Readings {
    pub const fn new() -> Self {
        Self {
            pa0: 0,
            pa1: 0,
            pa2: 0,
            pa3: 0,
            pa4: 0,
            pa5: 0,
            pa6: 0,
            pa7: 0,
        }
    }
}

/// Implement `embedded_dma::StaticWriteBuffer` automatically.
unsafe impl WriteTarget for Adc1Readings {
    type Word = u16;
}

#[derive(Debug, defmt::Format, Default, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct Adc2Readings {
    /// ADC12_IN8
    pub pb0: u16,
    /// ADC12_IN9
    pub pb1: u16,
    /// ADC123_IN10
    pub pc0: u16,
    /// ADC123_IN11
    pub pc1: u16,
    /// ADC123_IN12
    pub pc2: u16,
    /// ADC123_IN13
    pub pc3: u16,
    /// ADC12_IN14
    pub pc4: u16,
    /// ADC12_IN15
    pub pc5: u16,
}

impl Adc2Readings {
    pub const fn new() -> Self {
        Self {
            pb0: 0,
            pb1: 0,
            pc0: 0,
            pc1: 0,
            pc2: 0,
            pc3: 0,
            pc4: 0,
            pc5: 0,
        }
    }
}

/// Implement `embedded_dma::StaticWriteBuffer` automatically.
unsafe impl WriteTarget for Adc2Readings {
    type Word = u16;
}

#[derive(Debug, defmt::Format, Default, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct Adc3Readings {
    /// ADC3_IN9
    pub pf3: u16,
    /// ADC3_IN14
    pub pf4: u16,
    /// ADC3_IN15
    pub pf5: u16,
    /// ADC3_IN4
    pub pf6: u16,
    /// ADC3_IN5
    pub pf7: u16,
    /// ADC3_IN6
    pub pf8: u16,
    /// ADC3_IN7
    pub pf9: u16,
    /// ADC3_IN8
    pub pf10: u16,
}

impl Adc3Readings {
    pub const fn new() -> Self {
        Self {
            pf3: 0,
            pf4: 0,
            pf5: 0,
            pf6: 0,
            pf7: 0,
            pf8: 0,
            pf9: 0,
            pf10: 0,
        }
    }
}

/// Implement `embedded_dma::StaticWriteBuffer` automatically.
unsafe impl WriteTarget for Adc3Readings {
    type Word = u16;
}
