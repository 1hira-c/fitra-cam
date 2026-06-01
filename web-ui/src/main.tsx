import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { BrowserRouter, Routes, Route, Navigate } from "react-router-dom";
import { ViewerPage } from "./routes/ViewerPage";
import { SubjectCalibPage } from "./routes/SubjectCalibPage";
import "./styles/global.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<ViewerPage />} />
        <Route path="/subject-calib" element={<SubjectCalibPage />} />
        {/* Future: <Route path="/vmt-manager" element={<VmtManagerPage />} /> */}
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
    </BrowserRouter>
  </StrictMode>,
);
