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
    G(G: Nav & SLAM Algorithms - 6d)
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
