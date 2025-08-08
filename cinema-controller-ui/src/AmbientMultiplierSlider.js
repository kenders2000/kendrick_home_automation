import React, { useState } from "react";

function AmbientMultiplierSlider() {
  const [value, setValue] = useState(1.0);

  const handleChange = async (e) => {
    const newValue = parseFloat(e.target.value);
    setValue(newValue);

    try {
      await fetch(`http://localhost:8000/ambient_multiplier/${newValue}`, {
        method: "GET",
      });
      console.log("Ambient multiplier updated:", newValue);
    } catch (err) {
      console.error("Failed to update ambient multiplier:", err);
    }
  };

  return (
    <div>
      <label>Ambient Multiplier: {value.toFixed(2)}</label>
      <input
        type="range"
        min="0"
        max="1"
        step="0.01"
        value={value}
        onChange={handleChange}
      />
    </div>
  );
}

export default AmbientMultiplierSlider;
