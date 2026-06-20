// Thin layout wrapper that renders the WizardSteps bar above a step page. The
// live step comes from the polled daemon mode when known (so the bar tracks the
// auto-chain), else the page's own step. Flow-managed step clicks request a
// switch via useFlowSwitch.

import type { ReactNode } from "react";
import { useFlowSwitch } from "../hooks/useFlowSwitch";
import { MODE_FOR_STEP, STEP_FOR_MODE, type WizardStep } from "../lib/wizard";
import { WizardSteps } from "./WizardSteps";
import type { FlowState } from "../types/bundle";

interface Props {
  current: WizardStep;
  flow?: FlowState | null;
  children: ReactNode;
}

export function WizardLayout({ current, flow, children }: Props) {
  const { switchTo, banner } = useFlowSwitch();
  const managed = !!flow?.managed;
  const liveStep: WizardStep = flow?.mode ? STEP_FOR_MODE[flow.mode] : current;
  return (
    <>
      <WizardSteps
        current={liveStep}
        managed={managed}
        onSwitch={(step, label) => switchTo(MODE_FOR_STEP[step], label)}
      />
      {banner && <div className={`wizard-switch-banner ${banner.cls}`.trim()}>{banner.text}</div>}
      {children}
    </>
  );
}
