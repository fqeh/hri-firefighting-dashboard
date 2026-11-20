import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


def load_equipment_registry(registry_path: Path, world_path: Path) -> dict[str, Any]:
    registry_path = Path(registry_path)
    world_path = Path(world_path)
    with registry_path.open(encoding="utf-8") as file:
        registry = json.load(file)
    if not isinstance(registry, dict) or registry.get("schema_version") != 1:
        raise ValueError("Unsupported equipment registry schema")
    entries = registry.get("equipment")
    if not isinstance(entries, list) or not entries:
        raise ValueError("Equipment registry must contain a non-empty equipment list")

    models = {
        model.get("name"): model
        for model in ET.parse(world_path).getroot().findall(".//model")
    }
    equipment_ids: set[str] = set()
    gazebo_models: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("Every equipment registry entry must be an object")
        equipment_id = entry.get("equipment_id")
        gazebo_model = entry.get("gazebo_model")
        if not isinstance(equipment_id, str) or not equipment_id:
            raise ValueError("Every registry entry must have an equipment_id")
        if equipment_id in equipment_ids:
            raise ValueError(f"Duplicate equipment_id: {equipment_id}")
        if not isinstance(gazebo_model, str) or gazebo_model not in models:
            raise ValueError(
                f"Equipment {equipment_id} references missing Gazebo model {gazebo_model!r}"
            )
        if gazebo_model in gazebo_models:
            raise ValueError(f"Duplicate Gazebo model reference: {gazebo_model}")
        if entry.get("geometry_reference") != f"oil_facility.sdf::{gazebo_model}":
            raise ValueError(f"Equipment {equipment_id} has an invalid geometry_reference")
        equipment_ids.add(equipment_id)
        gazebo_models.add(gazebo_model)
        position = entry.get("position_m")
        if (
            not isinstance(position, list)
            or len(position) != 3
            or any(type(coordinate) not in (int, float) for coordinate in position)
            or any(not math.isfinite(coordinate) for coordinate in position)
        ):
            raise ValueError(f"Equipment {equipment_id} must have a 3D position_m")
        model_pose = models[gazebo_model].findtext("pose", "0 0 0 0 0 0").split()
        if len(model_pose) < 3 or any(
            abs(float(model_pose[index]) - float(position[index])) > 1e-3
            for index in range(3)
        ):
            raise ValueError(
                f"Equipment {equipment_id} position does not match {gazebo_model} in Gazebo"
            )
        contents = entry.get("contents")
        if not isinstance(contents, dict) or any(
            not isinstance(contents.get(field), str)
            for field in ("label", "material_class", "source")
        ):
            raise ValueError(f"Equipment {equipment_id} has invalid contents metadata")
    registry["source"] = "configuration"
    return registry
