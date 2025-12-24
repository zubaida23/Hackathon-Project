# ADR 1: ROS 2 Distribution Selection

## Status
Accepted

## Context
For the Physical AI & Humanoid Robotics project, we need to select a ROS 2 distribution that will provide stability, long-term support, and compatibility with the project's requirements for a spec-driven technical book. The choice affects all modules that depend on ROS 2 functionality.

## Decision
We will use ROS 2 Humble Hawksbill LTS (Long-Term Support) as the primary ROS 2 distribution for this project.

## Options Considered

### ROS 2 Humble Hawksbill (LTS)
- Released in May 2022
- Support until May 2027 (5-year support cycle)
- Extensive documentation and community support
- Compatible with Python 3.8+ and Ubuntu 22.04
- Well-tested with industrial applications

### ROS 2 Rolling Ridley
- Latest features and updates
- Shorter support cycle
- Potential instability for educational content
- May break compatibility frequently

### ROS 2 Iron Irwini
- Released in May 2023
- Support until November 2024 (1.5-year support cycle)
- Newer features but shorter support than Humble
- Good middle ground but limited long-term support

## Rationale
Humble Hawksbill provides the best balance of stability, long-term support, and compatibility for an educational project. The 5-year support cycle ensures that examples and tutorials will remain valid throughout the book's lifecycle, which is critical for reproducibility as specified in the project constitution. The extensive documentation and community support make it ideal for beginners and intermediate learners.

## Consequences

### Positive
- Long-term stability for educational content
- Extensive documentation available for learners
- Large community for support and troubleshooting
- Compatible with all required tools and libraries
- Reduces need for updates during the book's lifecycle

### Negative
- May miss some newer features available in Rolling
- Requires Ubuntu 22.04 or similar supported platforms
- Some cutting-edge packages might target newer distributions

## Alternatives
Considered Rolling Ridley for latest features and Iron Irwini as a compromise, but both were rejected due to shorter support cycles which would compromise the reproducibility requirement.

## Links
- Related to: specs/humanoid-robotics/spec.md
- Implementation: specs/humanoid-robotics/tasks.md (Task 1.1)