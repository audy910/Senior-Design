# Senior-Design: Autonomous Service Robot

## 1. Critical Path Method Strategy
```mermaid
graph TD
    A(Procure Components) --> B(Chassis & Motor Mount)
    A --> E(NPU CV Training)
    A --> F(Audio I/O Setup)
    B --> C(CAN Bus Logic)
    C --> D(Sensor Integration)
    D --> G(GPS & Path Planning)
    E --> G
    G --> H(System Integration)
    F --> H
    H --> I(Field Testing)

    style A stroke:#f00,stroke-width:4px
    style B stroke:#f00,stroke-width:4px
    style C stroke:#f00,stroke-width:4px
    style D stroke:#f00,stroke-width:4px
    style G stroke:#f00,stroke-width:4px
    style H stroke:#f00,stroke-width:4px
    style I stroke:#f00,stroke-width:4px
gantt
    title Robot Development Timeline
    dateFormat  YYYY-MM-DD
    section Hardware
    Procure Components           :a1, 2026-03-01, 5d
    Chassis Assembly             :a2, after a1, 3d
    CAN Bus Logic                :a3, after a2, 4d
    Sensor Integration           :a4, after a3, 3d
    section AI & Nav
    NPU CV Training              :b1, after a1, 7d
    GPS & Path Planning          :b2, after a4, 6d
    section Audio
    Audio IO Setup               :c1, after a1, 3d
    section Final
    System Integration           :d1, after b2, 5d
    Field Testing                :d2, after d1, 4d
