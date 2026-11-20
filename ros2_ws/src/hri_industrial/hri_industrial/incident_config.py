import copy
import json
import math
from pathlib import Path
from typing import Any, Callable


def load_tank_damage_incident(path: str | Path, layout: dict[str, Any]) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as file:
        catalog = json.load(file)
    if not isinstance(catalog, dict) or catalog.get("schema_version") != 1:
        raise ValueError("Unsupported industrial incident catalog schema")
    incidents = catalog.get("incidents")
    if not isinstance(incidents, list) or len(incidents) != 1:
        raise ValueError("Expected one configured industrial aftermath incident")
    incident = incidents[0]
    if not isinstance(incident, dict) or incident.get("event_type") != "tank_damage_aftermath":
        raise ValueError("Invalid tank damage aftermath event")
    for field in ("event_id", "display_name", "equipment_id", "tank_model", "visual_entity", "visual_model"):
        if not isinstance(incident.get(field), str) or not incident[field]:
            raise ValueError(f"Incident must define {field}")
    if incident["tank_model"] != "tank_0" or incident["equipment_id"] != "TK-01":
        raise ValueError("Tank aftermath must reference configured equipment TK-01")
    if incident["visual_entity"] != "tank_0_damage_visual":
        raise ValueError("Unexpected damage visual entity name")
    visual_position = incident.get("visual_position")
    if not _valid_position(visual_position):
        raise ValueError("Incident visual_position must contain three finite coordinates")
    hazard = incident.get("hazard")
    if not isinstance(hazard, dict):
        raise ValueError("Incident must define its route hazard")
    if (hazard.get("kind") != "debris" or hazard.get("model") != "hazard_debris"
            or not isinstance(hazard.get("id"), str) or not hazard["id"].startswith("hazard_debris_")
            or hazard.get("edge") not in {edge["id"] for edge in layout.get("edges", [])}):
        raise ValueError("Incident route hazard must be debris on a surveyed facility edge")
    if not _valid_position(hazard.get("position")):
        raise ValueError("Incident route hazard position must contain three finite coordinates")
    if type(hazard.get("radius")) not in (int, float) or not math.isfinite(hazard["radius"]) or hazard["radius"] <= 0:
        raise ValueError("Incident route hazard radius must be positive and finite")
    if incident.get("manual_only") is not True:
        raise ValueError("Industrial aftermath must be manually triggered")
    note = incident.get("aftermath_note")
    if not isinstance(note, str) or any(
        required not in note.lower()
        for required in ("fictional", "operator-triggered", "no impact dynamics", "no combustion",
                         "no material release", "no dispersion", "no equipment process telemetry")
    ):
        raise ValueError("Incident limitations must remain explicit")
    return incident


def tank_damage_hazard(incident: dict[str, Any]) -> dict[str, Any]:
    hazard = copy.deepcopy(incident["hazard"])
    return {
        **hazard,
        "event_id": incident["event_id"],
        "event_type": incident["event_type"],
        "display_name": incident["display_name"],
        "equipment_id": incident["equipment_id"],
        "tank_model": incident["tank_model"],
        "visual_entity": incident["visual_entity"],
        "visual_model": incident["visual_model"],
        "visual_position": list(incident["visual_position"]),
        "manual_only": True,
        "at": 1_000_000_000.0,
    }


def add_tank_damage_hazard(config: dict[str, Any], incident: dict[str, Any]) -> dict[str, Any]:
    updated = copy.deepcopy(config)
    hazards = updated.get("hazards")
    if not isinstance(hazards, list):
        raise ValueError("Scenario must define a hazard list")
    event_hazard = tank_damage_hazard(incident)
    if any(not isinstance(item, dict) or item.get("id") == event_hazard["id"] for item in hazards):
        raise ValueError("Tank aftermath hazard ID already exists or scenario hazards are invalid")
    hazards.append(event_hazard)
    return updated


def execute_tank_damage_aftermath(
    hazard: dict[str, Any],
    industrial_models: Path,
    hazard_models: Path,
    spawn_entity: Callable[[Path, str, list[float], float], bool],
    publish_event: Callable[[str, dict[str, Any]], None],
    completed_ids: set[str],
    edge_yaw: float,
) -> bool:
    if hazard["id"] in completed_ids:
        return False
    event_id = hazard["event_id"]
    equipment_id = hazard["equipment_id"]
    publish_event("/hri/events", {
        "source": "operator",
        "event": "tank_damage_aftermath_requested",
        "incident_id": event_id,
        "equipment_id": equipment_id,
        "detail": "Operator requested the predefined visual aftermath; no impact or process telemetry is modeled.",
    })
    try:
        spawn_entity(
            Path(industrial_models) / hazard["visual_model"] / "model.sdf",
            hazard["visual_entity"],
            hazard["visual_position"],
            0.0,
        )
        spawn_entity(
            Path(hazard_models) / hazard["model"] / "model.sdf",
            hazard["id"],
            hazard["position"],
            edge_yaw,
        )
    except (OSError, RuntimeError, ValueError, TypeError) as exc:
        publish_event("/hri/events", {
            "source": "industrial",
            "event": "tank_damage_aftermath_failed",
            "incident_id": event_id,
            "equipment_id": equipment_id,
            "error": str(exc),
        })
        raise
    completed_ids.add(hazard["id"])
    publish_event("/hri/events", {
        "source": "industrial",
        "event": "tank_damage_aftermath_created",
        "incident_id": event_id,
        "equipment_id": equipment_id,
        "visual_entity": hazard["visual_entity"],
        "debris_entity": hazard["id"],
        "affected_edge": hazard["edge"],
        "debris_position": hazard["position"],
        "hazard_note": "Configured physical debris; route status changes only after robot sensor observation.",
        "limitations": "Fictional aftermath only. No impact dynamics, combustion, material release, dispersion, temperature, pressure, inventory, or integrity measurement.",
    })
    return True


def _valid_position(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(type(coordinate) in (int, float) and math.isfinite(coordinate) for coordinate in value)
    )
