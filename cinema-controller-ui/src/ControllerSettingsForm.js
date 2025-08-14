import React, { useEffect, useState } from "react";

// 1) Point this at your Pi when developing from another machine
const API_BASE =
  import.meta?.env?.VITE_API_BASE ||
  process.env.REACT_APP_API_BASE ||
  window.location.origin; // works if you serve the UI from the same FastAPI

export default function ControllerSettingsForm() {
  const [settings, setSettings] = useState(null);
  const [filename, setFilename] = useState("settings.json");
  const [error, setError] = useState("");

  useEffect(() => {
    fetch(`${API_BASE}/settings`)
      .then((res) => {
        if (!res.ok) throw new Error(`GET /settings failed: HTTP ${res.status}`);
        return res.json();
      })
      .then((json) => {
        // Helpful for debugging
        console.log("Settings JSON:", json);
        setSettings(json);
        setError("");
      })
      .catch((err) => {
        console.error(err);
        setError(String(err));
      });
  }, []);

  if (!settings) {
    return (
      <div>
        Loading…
        {error && <div style={{ color: "crimson" }}>Error: {error}</div>}
      </div>
    );
  }

  const handleChange = (e) => {
    const { name, value, type } = e.target;
    // Keep numbers as numbers; allow empty string while typing
    const next =
      type === "number" && value !== "" ? Number(value) : value;
    setSettings((prev) => ({ ...prev, [name]: next }));
  };

  const handleSubmit = () => {
    fetch(`${API_BASE}/settings`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(settings),
    })
      .then((res) => res.json())
      .then((data) => {
        alert(data.message || "Settings updated & controller restarted");
      })
      .catch((err) => {
        console.error(err);
        alert("Failed to update settings");
      });
  };

  // /save_settings expects ?filename=... (query param), not a path segment
  const saveToFile = () => {
    const url = `${API_BASE}/save_settings?filename=${encodeURIComponent(
      filename
    )}`;
    fetch(url, { method: "POST" })
      .then((res) => res.json())
      .then((data) => alert(data.message || `Saved to ${data.path || filename}`))
      .catch((err) => {
        console.error(err);
        alert("Failed to save settings");
      });
  };

  // Your backend doesn’t have /load_settings/<file> — it has /reload_settings
  // which loads settings from "settings.json". Either change the backend
  // (see note below) or just call /reload_settings here.
  const reloadFromFile = () => {
    fetch(`${API_BASE}/reload_settings`, { method: "POST" })
      .then((res) => res.json())
      .then((data) => {
        // Re-pull settings so the form reflects what the server now uses
        return fetch(`${API_BASE}/settings`)
          .then((r) => r.json())
          .then(setSettings)
          .then(() => alert(data.message || "Settings reloaded"));
      })
      .catch((err) => {
        console.error(err);
        alert("Failed to reload settings");
      });
  };

  return (
    <div style={{ maxWidth: 720 }}>
      <h2>Controller Settings</h2>

      {Object.entries(settings).map(([key, value]) => (
        <div key={key} style={{ marginBottom: 10 }}>
          <label style={{ display: "block", fontWeight: 600 }}>{key}</label>
          <input
            name={key}
            type={typeof value === "number" ? "number" : "text"}
            value={value ?? ""}        // keep controlled
            step="any"
            onChange={handleChange}
            style={{ width: "100%" }}
          />
        </div>
      ))}

      <button onClick={handleSubmit}>Update Settings</button>

      <hr />

      <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
        <input
          type="text"
          value={filename}
          onChange={(e) => setFilename(e.target.value)}
          placeholder="settings filename (e.g. settings.json)"
          style={{ flex: 1 }}
        />
        <button onClick={saveToFile}>Save Settings</button>
        <button onClick={reloadFromFile}>Reload (server’s default)</button>
      </div>
    </div>
  );
}
