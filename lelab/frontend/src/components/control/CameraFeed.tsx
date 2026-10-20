// Modified for SO101 Arm Control; see the repository THIRD_PARTY.md.
import React from "react";
import BrowserCameraPreview from "./BrowserCameraPreview";
import { CameraRotation } from "@/lib/cameraConfig";

interface CameraFeedProps {
  deviceId: string;
  label?: string;
  rotation?: CameraRotation;
  width?: number;
  height?: number;
  fps?: number;
}

const CameraFeed: React.FC<CameraFeedProps> = ({ deviceId, label, rotation, width, height, fps }) => (
  <div className="bg-gray-900 rounded-lg border border-gray-700 overflow-hidden">
    <BrowserCameraPreview deviceId={deviceId} rotation={rotation} width={width} height={height} fps={fps} />
    {label && (
      <div className="p-2 text-sm text-gray-300 truncate border-t border-gray-800">{label}</div>
    )}
  </div>
);

export default CameraFeed;
