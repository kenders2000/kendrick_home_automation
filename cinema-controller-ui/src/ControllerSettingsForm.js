import React, { useEffect, useState } from "react";

function ControllerSettingsForm() {
  const [settings, setSettings] = useState(null);
  const [filename, setFilename] = useState("default.json");

  useEffect(() => {
    fetch("http://localhost:8000/settings")
      .then((res) => res.json())
      .then(setSettings)
      .catch((err) => console.error("Failed to fetch settings:", err));
  }, []);

  const handleChange = (e) => {
    const { name, value, type } = e.target;
    const parsedValue = type === "number" ? parseFloat(value) : value;
    setSettings((prev) => ({ ...prev, [name]: parsedValue }));
  };

  const handleSubmit = () => {
    fetch("http://localhost:8000/settings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(settings),
    })
      .then((res) => res.json())
      .then((data) => alert(data.message || "Settings updated"))
      .catch((err) => console.error("Failed to update settings:", err));
  };

  const saveToFile = () => {
    fetch(`http://localhost:8000/save_settings/${filename}`, { method: "POST" })
      .then((res) => res.json())
      .then((data) => alert(data.message || "Settings saved"))
      .catch((err) => console.error("Failed to save settings:", err));
  };

  const loadFromFile = () => {
    fetch(`http://localhost:8000/load_settings/${filename}`, { method: "POST" })
      .then((res) => res.json())
      .then(() => window.location.reload())
      .catch((err) => console.error("Failed to load settings:", err));
  };

  if (!settings) return <div>Loading...</div>;

  return (
    <div>
      <h2>Controller Settings</h2>
      {Object.entries(settings).map(([key, value]) => (
        <div key={key} style={{ marginBottom: "10px" }}>
          <label>
            {key}: <br />
            <input
              name={key}
              type={typeof value === "number" ? "number" : "text"}
              value={value}
              onChange={handleChange}
              step="any"
            />
          </label>
        </div>
      ))}
      <button onClick={handleSubmit}>Update Settings</button>
      <hr />
      <input
        type="text"
        value={filename}
        onChange={(e) => setFilename(e.target.value)}
        placeholder="settings filename"
      />
      <button onClick={saveToFile}>Save Settings</button>
      <button onClick={loadFromFile}>Load Settings</button>
    </div>
  );
}

export default ControllerSettingsForm;
