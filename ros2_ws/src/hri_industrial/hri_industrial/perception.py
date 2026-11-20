"""Calibrate semantic footprints for the industrial assets; sensors stay Gazebo-owned."""
import json
from pathlib import Path
from hri_reasoning.tom_runtime import PublicPerception
from hri_reasoning.lifecycle import run


class IndustrialPerception(PublicPerception):
    def __init__(self):
        super().__init__()
        self.layout=json.loads(Path(self.parameter('facility_layout','')).read_text())

    def publish_json(self,topic,data):
        if topic=='/hri/observations':
            for hazard in data['hazards']:
                hazard['radius']=self.layout['hazard_radii'][hazard['kind']]
                hazard['sensor_assumption']='Ideal semantic hazard identity and pose; gas is a detector surrogate, not RGB recognition'
        super().publish_json(topic,data)


def main():run(IndustrialPerception)
