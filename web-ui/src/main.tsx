import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { BrowserRouter, Routes, Route, Navigate } from "react-router-dom";
import { ViewerPage } from "./routes/ViewerPage";
import { SubjectCalibPage } from "./routes/SubjectCalibPage";
import { IntrinsicCalibPage } from "./routes/IntrinsicCalibPage";
import { ExtrinsicCalibPage } from "./routes/ExtrinsicCalibPage";
import { SetupPage } from "./routes/SetupPage";
import "./styles/global.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<ViewerPage />} />
        <Route path="/setup" element={<SetupPage />} />
        <Route path="/intrinsic-calib" element={<IntrinsicCalibPage />} />
        <Route path="/extrinsic-calib" element={<ExtrinsicCalibPage />} />
        <Route path="/subject-calib" element={<SubjectCalibPage />} />
        {/* Future: <Route path="/vmt-manager" element={<VmtManagerPage />} /> */}
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
    </BrowserRouter>
  </StrictMode>,
);
