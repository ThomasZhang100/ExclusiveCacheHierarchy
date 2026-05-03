# Slideshow Outline

## Slide 1: Title and Research Question
- Title: Exclusive vs. Inclusive Cache Hierarchies Under Conflict-Sensitive Workloads
- One-sentence question: Which hierarchy performs better, and how much does cache configuration matter?
- Presenters: split opener and roadmap here

## Slide 2: System Design
- Show the riscvlong core feeding either the exclusive or inclusive 3-level hierarchy
- Explain the difference:
  Exclusive: a line lives in exactly one level
  Inclusive: L1 contents are replicated in lower levels
- Optional hand-drawn diagram here is totally fine and may look better than a dense RTL screenshot

## Slide 3: Methodology
- Benchmarks: vvadd, cmplx-mult, masked-filter, bin-search, conflict-miss
- Metrics: cycles and IPC
- Config sweep: default, L1 half/double, victim-buffer attempt, L2 assoc, L3 size
- Mention that 80 real simulation runs were collected

## Slide 4: Baseline Results
- Use figure: deliverables/figures/baseline_cycles_5bench.png
- Main takeaway:
  Exclusive is slightly better on the original four benchmarks
  Inclusive is much better on conflict-miss

## Slide 5: L1 Capacity Dominates Conflict-Miss
- Use figure: deliverables/figures/conflict_miss_l1_focus.png
- Main takeaway:
  Doubling L1 drops conflict-miss cycles dramatically for both hierarchies
  Once L1 is larger, the exclusive/inclusive gap nearly disappears

## Slide 6: Full Sweep and Conclusions
- Use figure: deliverables/figures/conflict_miss_sweep.png
- Main takeaway:
  L2 associativity and L3 size barely move results for this benchmark set
  L1 behavior is the dominant factor
  Workload matters: hierarchy choice depends on conflict sensitivity

## Suggested Presenter Split
- Person 1:
  Slides 1 to 3
  Motivation, design, methodology
- Person 2:
  Slides 4 to 6
  Results, interpretation, conclusion

## Figure Notes
- Yes, figures were created.
- Recommended slideshow figures:
  1. baseline_cycles_5bench.png
  2. conflict_miss_l1_focus.png
  3. conflict_miss_sweep.png
- If you want a more hand-made aesthetic, keep the generated charts for results and hand-draw only the architecture diagram on the iPad.
