# Senior-Design

# Autonomous Service Robot: Project Schedule (CPM)

This diagram tracks the **Critical Path Method (CPM)** for our robot development. Tasks outlined in **red** represent the critical path—any delay in these tasks will push back the entire project completion date.

```mermaid
graph TD
    %% Define Tasks
    A(A: Procure Components - 5d)
    B(B: Chassis & Motor Mount - 3d)
    C(C: CAN Bus Logic & Wiring - 4d)
    D(D: Sensor Integration - 3d)
    E(E: NPU CV Model Training - 7d)
    F(F: Audio I/O Setup - 3d)
    G(G: GPS/Path Planning - 6d)
    H(H: System Integration - 5d)
    I(I: Field Testing - 4d)

    %% Define Dependencies
    A --> B
    A --> E
    A --> F
    B --> C
    C --> D
    D --> G
    E --> G
    G --> H
    F --> H
    H --> I

    %% Styling the Critical Path (Red = High Priority)
    style A stroke:#f00,stroke-width:4px
    style B stroke:#f00,stroke-width:4px
    style C stroke:#f00,stroke-width:4px
    style D stroke:#f00,stroke-width:4px
    style G stroke:#f00,stroke-width:4px
    style H stroke:#f00,stroke-width:4px
    style I stroke:#f00,stroke-width:4px

    subgraph Legend
        CP[Red Border = Critical Path]
    end


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
