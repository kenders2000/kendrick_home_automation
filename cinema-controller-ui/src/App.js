import React from "react";
import AmbientMultiplierSlider from "./AmbientMultiplierSlider";
import SaveLoadButtons from "./SaveLoadButtons";
import ControllerSettingsForm from "./ControllerSettingsForm";

function App() {
  return (
    <div>
      <h1>Lighting Control UI</h1>
      <AmbientMultiplierSlider />
      <SaveLoadButtons />
      <ControllerSettingsForm />
    </div>
  );
}

export default App;
