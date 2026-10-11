# Architecture Decision Records (ADRs)

Architecture Decision Records (ADRs) capture significant architectural decisions along with their context, tradeoffs, and consequences.

We follow the [Michael Nygard ADR format](https://github.com/joelparkerhenderson/architecture-decision-record) to maintain an immutable log of engineering rationale.

---

## 📋 ADR Catalog

| ADR | Title | Status | Date |
| :--- | :--- | :---: | :---: |
| [**ADR-001**](ADR-001-async-to-edge-inference-evolution.md) | Two-Stage Evolution from Remote Async Inference to On-Board Hailo-8 Edge Inference | Accepted | 2026-10-08 |

---

## 📝 ADR Template

When authoring a new ADR, create a file named `ADR-XXX-<short-title>.md` using the following structure:

```markdown
# ADR-[Number]: [Short Title of Decision]

## Status
[Proposed | Accepted | Deprecated | Superseded by ADR-XXX]

## Context
[What forces are at play? Technical, organizational, physical? What constraints must be met?]

## Decision
[What is the change we are proposing or have agreed to?]

## Consequences
- **Positive**: [What becomes easier or better?]
- **Negative / Tradeoffs**: [What becomes harder or what constraints are accepted?]
- **Neutral**: [What changes but is neither better nor worse?]

## Alternatives Considered
- **Option 1**: [Description, Pros, Cons]

## References
- [Links to issues, RFCs, benchmarks, or specs]
```

