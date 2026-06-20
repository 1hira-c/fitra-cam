// Presentational setup-progress stepper. Shows the ordered wizard steps with
// done / current / upcoming styling. The current step is derived from the live
// daemon mode by the parent (WizardLayout). When `onSwitch` is given and the
// daemon is flow-managed, clicking another step requests a flow-switch;
// otherwise steps are plain route links.

import { Link } from "react-router-dom";
import {
  ROUTE_FOR_STEP,
  STEP_LABELS,
  STEP_ORDER,
  type WizardStep,
} from "../lib/wizard";
import "../styles/wizard.css";

interface Props {
  current: WizardStep | null;
  managed?: boolean;
  onSwitch?: (step: WizardStep, label: string) => void;
}

export function WizardSteps({ current, managed, onSwitch }: Props) {
  const curIdx = current ? STEP_ORDER.indexOf(current) : -1;
  return (
    <nav className="wizard-steps" aria-label="setup progress">
      {STEP_ORDER.map((step, i) => {
        const state =
          curIdx < 0 ? "upcoming" : i < curIdx ? "done" : i === curIdx ? "current" : "upcoming";
        const label = STEP_LABELS[step];
        const cls = `wizard-step ${state}`;
        const body = (
          <>
            <span className="wizard-step-num">{i + 1}</span>
            <span className="wizard-step-label">{label}</span>
          </>
        );
        // Flow-managed: switching steps means asking the daemon to respawn into
        // that mode. Otherwise a plain link (manual / standalone).
        if (onSwitch && managed && step !== current) {
          return (
            <button key={step} type="button" className={cls} onClick={() => onSwitch(step, label)}>
              {body}
            </button>
          );
        }
        return (
          <Link key={step} className={cls} to={ROUTE_FOR_STEP[step]}>
            {body}
          </Link>
        );
      })}
    </nav>
  );
}
