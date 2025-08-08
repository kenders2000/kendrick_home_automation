import React, { useState } from "react";

function SaveLoadButtons() {
  const [filename, setFilename] = useState("default.json");

  const saveSettings = async () => {
    try {
      await fetch(`http://localhost:8000/save_settings/${filename}`, {
        method: "POST",
      });
      alert(`Saved to ${filename}`);
    } catch (err) {
      console.error("Save failed:", err);
    }
  };

  const loadSettings = async () => {
    try {
      await fetch(`http://localhost:8000/load_settings/${filename}`, {
        method: "POST",
      });
      alert(`Loaded ${filename}`);
    } catch (err) {
      console.error("Load failed:", err);
    }
  };

  return (
    <div>
      <input
        type="text"
        value={filename}
        onChange={(e) => setFilename(e.target.value)}
        placeholder="settings filename"
      />
      <button onClick={saveSettings}>Save Settings</button>
      <button onClick={loadSettings}>Load Settings</button>
    </div>
  );
}

export default SaveLoadButtons;
