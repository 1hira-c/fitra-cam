// Setup-wizard step model (docs/design/core-pipeline-setup-mode.md). The wizard
// REFLECTS the flow daemon's current mode; it does not drive the auto-chain.
// Keep STEP_FOR_MODE in sync with PAGE_FOR_MODE in hooks/useFlowWatch.ts.

import type { FlowMode } from "../types/bundle";

export type WizardStep = "setup" | "intrinsic" | "extrinsic" | "subject" | "run";

export const STEP_ORDER: WizardStep[] = [
  "setup",
  "intrinsic",
  "extrinsic",
  "subject",
  "run",
];

export const STEP_LABELS: Record<WizardStep, string> = {
  setup: "セットアップ",
  intrinsic: "内部校正",
  extrinsic: "外部校正",
  subject: "被験者校正",
  run: "推論",
};

// Daemon mode -> wizard step. Both extrinsic variants collapse onto one step.
export const STEP_FOR_MODE: Record<FlowMode, WizardStep> = {
  setup: "setup",
  "calib-intrinsic": "intrinsic",
  "calib-extrinsic": "extrinsic",
  "calib-extrinsic-floor": "extrinsic",
  "calib-subject": "subject",
  run: "run",
};

// Wizard step -> SPA route.
export const ROUTE_FOR_STEP: Record<WizardStep, string> = {
  setup: "/setup",
  intrinsic: "/intrinsic-calib",
  extrinsic: "/extrinsic-calib",
  subject: "/subject-calib",
  run: "/",
};

// Wizard step -> the daemon mode a flow-switch should request for that step.
export const MODE_FOR_STEP: Record<WizardStep, FlowMode> = {
  setup: "setup",
  intrinsic: "calib-intrinsic",
  extrinsic: "calib-extrinsic",
  subject: "calib-subject",
  run: "run",
};
