#!/usr/bin/env python3
"""
Convert bounding box CSV data to IFC format.

The CSV contains wall bounding boxes with columns:
min_x, min_y, min_z, max_x, max_y, max_z, center_x, center_y, center_z,
dx, dy, dz, wall_id, ply_file, n_points

Based on hybrid_ifc_generator.py using ifcopenshell for proper IFC4 structure.
"""

import csv
import time
from pathlib import Path

try:
    import ifcopenshell
    import ifcopenshell.guid
except ImportError:
    print("[ERROR] ifcopenshell not installed. Install with: pip install ifcopenshell")
    import sys
    sys.exit(1)
try:
    import ifcopenshell
    import ifcopenshell.guid
except ImportError:
    print("[ERROR] ifcopenshell not installed. Install with: pip install ifcopenshell")
    import sys
    sys.exit(1)


def create_ifc_project(project_name: str = "Wall Export"):
    """
    Create IFC4 project structure with proper schema.
    Based on hybrid_ifc_generator.py
    
    Returns:
        Tuple of (ifc, storey, body_context, owner_history)
    """
    # Create IFC file
    ifc = ifcopenshell.file(schema="IFC4")
    
    # === Person and Organization ===
    person = ifc.create_entity("IfcPerson")
    org = ifc.create_entity("IfcOrganization", Name="BBox to IFC Converter")
    user = ifc.create_entity(
        "IfcPersonAndOrganization",
        ThePerson=person,
        TheOrganization=org
    )
    
    # === Application ===
    app = ifc.create_entity(
        "IfcApplication",
        ApplicationDeveloper=org,
        Version="1.0",
        ApplicationFullName="BBox to IFC Converter",
        ApplicationIdentifier="BBOX2IFC"
    )
    
    # === Owner History ===
    owner_history = ifc.create_entity(
        "IfcOwnerHistory",
        OwningUser=user,
        OwningApplication=app,
        ChangeAction="ADDED",
        CreationDate=int(time.time())
    )
    
    # === Unit System ===
    length_unit = ifc.create_entity(
        "IfcSIUnit",
        UnitType="LENGTHUNIT",
        Name="METRE"
    )
    area_unit = ifc.create_entity(
        "IfcSIUnit",
        UnitType="AREAUNIT",
        Name="SQUARE_METRE"
    )
    volume_unit = ifc.create_entity(
        "IfcSIUnit",
        UnitType="VOLUMEUNIT",
        Name="CUBIC_METRE"
    )
    angular_unit = ifc.create_entity(
        "IfcSIUnit",
        UnitType="PLANEANGLEUNIT",
        Name="RADIAN"
    )
    
    units = ifc.create_entity(
        "IfcUnitAssignment",
        Units=[length_unit, area_unit, volume_unit, angular_unit]
    )
    
    # === Geometric Coordinate System ===
    origin = ifc.create_entity(
        "IfcCartesianPoint",
        Coordinates=[0.0, 0.0, 0.0]
    )
    x_axis = ifc.create_entity(
        "IfcDirection",
        DirectionRatios=[1.0, 0.0, 0.0]
    )
    z_axis = ifc.create_entity(
        "IfcDirection",
        DirectionRatios=[0.0, 0.0, 1.0]
    )
    
    placement = ifc.create_entity(
        "IfcAxis2Placement3D",
        Location=origin,
        Axis=z_axis,
        RefDirection=x_axis
    )
    
    # === Geometric Representation Context ===
    context = ifc.create_entity(
        "IfcGeometricRepresentationContext",
        ContextIdentifier="Model",
        ContextType="Model",
        CoordinateSpaceDimension=3,
        Precision=1e-5,
        WorldCoordinateSystem=placement
    )
    
    # === Body Subcontext ===
    body_context = ifc.create_entity(
        "IfcGeometricRepresentationSubContext",
        ContextIdentifier="Body",
        ContextType="Model",
        ParentContext=context,
        TargetView="MODEL_VIEW"
    )
    
    # === Project ===
    project = ifc.create_entity(
        "IfcProject",
        GlobalId=ifcopenshell.guid.new(),
        OwnerHistory=owner_history,
        Name=project_name,
        RepresentationContexts=[context],
        UnitsInContext=units
    )
    
    # === Spatial Structure ===
    site = ifc.create_entity(
        "IfcSite",
        GlobalId=ifcopenshell.guid.new(),
        OwnerHistory=owner_history,
        Name="Site"
    )
    
    building = ifc.create_entity(
        "IfcBuilding",
        GlobalId=ifcopenshell.guid.new(),
        OwnerHistory=owner_history,
        Name="Building"
    )
    
    storey = ifc.create_entity(
        "IfcBuildingStorey",
        GlobalId=ifcopenshell.guid.new(),
        OwnerHistory=owner_history,
        Name="Ground Floor",
        Elevation=0.0
    )
    
    # === Hierarchy Relationships ===
    ifc.create_entity(
        "IfcRelAggregates",
        GlobalId=ifcopenshell.guid.new(),
        OwnerHistory=owner_history,
        RelatingObject=project,
        RelatedObjects=[site]
    )
    
    ifc.create_entity(
        "IfcRelAggregates",
        GlobalId=ifcopenshell.guid.new(),
        OwnerHistory=owner_history,
        RelatingObject=site,
        RelatedObjects=[building]
    )
    
    ifc.create_entity(
        "IfcRelAggregates",
        GlobalId=ifcopenshell.guid.new(),
        OwnerHistory=owner_history,
        RelatingObject=building,
        RelatedObjects=[storey]
    )
    
    return ifc, storey, body_context, owner_history


def create_box_geometry(ifc, body_context, min_bound, max_bound):
    """
    Create IfcExtrudedAreaSolid geometry from bounding box.
    Based on hybrid_ifc_generator.py
    
    Args:
        ifc: IFC file object
        body_context: Body geometric context
        min_bound: [min_x, min_y, min_z]
        max_bound: [max_x, max_y, max_z]
        
    Returns:
        IfcProductDefinitionShape or None if invalid
    """
    # Calculate dimensions
    dx = abs(max_bound[0] - min_bound[0])
    dy = abs(max_bound[1] - min_bound[1])
    dz = abs(max_bound[2] - min_bound[2])
    
    # Validate dimensions
    if dx < 0.01 or dy < 0.01 or dz < 0.01:
        print(f"[WARN] Invalid dimensions: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
        return None
    
    # === Create 2D Profile (Rectangle) ===
    profile_origin = ifc.create_entity(
        "IfcCartesianPoint",
        Coordinates=[0.0, 0.0]
    )
    
    profile_axis = ifc.create_entity(
        "IfcAxis2Placement2D",
        Location=profile_origin
    )
    
    profile = ifc.create_entity(
        "IfcRectangleProfileDef",
        ProfileType="AREA",
        Position=profile_axis,
        XDim=float(dx),
        YDim=float(dy)
    )
    
    # === Create 3D Placement ===
    # IfcRectangleProfileDef uses center as origin, so offset by half dimensions
    center_offset_x = dx / 2.0
    center_offset_y = dy / 2.0
    
    base_point = ifc.create_entity(
        "IfcCartesianPoint",
        Coordinates=[
            float(min_bound[0] + center_offset_x),
            float(min_bound[1] + center_offset_y),
            float(min_bound[2])
        ]
    )
    
    z_dir = ifc.create_entity(
        "IfcDirection",
        DirectionRatios=[0.0, 0.0, 1.0]
    )
    
    x_dir = ifc.create_entity(
        "IfcDirection",
        DirectionRatios=[1.0, 0.0, 0.0]
    )
    
    axis3d = ifc.create_entity(
        "IfcAxis2Placement3D",
        Location=base_point,
        Axis=z_dir,
        RefDirection=x_dir
    )
    
    # === Create Extruded Area Solid ===
    extrude_dir = ifc.create_entity(
        "IfcDirection",
        DirectionRatios=[0.0, 0.0, 1.0]
    )
    
    solid = ifc.create_entity(
        "IfcExtrudedAreaSolid",
        SweptArea=profile,
        Position=axis3d,
        ExtrudedDirection=extrude_dir,
        Depth=float(dz)
    )
    
    # === Create Shape Representation ===
    body_rep = ifc.create_entity(
        "IfcShapeRepresentation",
        ContextOfItems=body_context,
        RepresentationIdentifier="Body",
        RepresentationType="SweptSolid",
        Items=[solid]
    )
    
    shape = ifc.create_entity(
        "IfcProductDefinitionShape",
        Representations=[body_rep]
    )
    
    return shape


def csv_bboxes_to_ifc(csv_path: Path, ifc_path: Path):
    """
    Convert bounding box CSV to IFC file using proper IFC4 structure.
    
    Args:
        csv_path: Path to input CSV file with bounding boxes
        ifc_path: Path to output IFC file
    """
    print(f"[INFO] Converting CSV to IFC: {csv_path}")
    
    # Create IFC project structure
    ifc, storey, body_context, owner_history = create_ifc_project("Wall Export")
    
    # Read CSV and create walls
    walls_added = 0
    wall_entities = []
    
    with open(csv_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            wall_entity = create_wall_from_bbox(ifc, body_context, owner_history, row)
            if wall_entity:
                wall_entities.append(wall_entity)
                walls_added += 1
    
    # Container relationship - attach all walls to storey
    if wall_entities:
        ifc.create_entity(
            "IfcRelContainedInSpatialStructure",
            GlobalId=ifcopenshell.guid.new(),
            OwnerHistory=owner_history,
            RelatingStructure=storey,
            RelatedElements=wall_entities
        )
    
    # Write IFC file
    ifc.write(str(ifc_path))
    file_size = ifc_path.stat().st_size
    
    print(f"[INFO] Created IFC file with {walls_added} walls: {ifc_path}")
    print(f"[INFO] File size: {file_size:,} bytes")


def create_wall_from_bbox(ifc, body_context, owner_history, row):
    """
    Create a wall entity from bounding box CSV data.
    Based on hybrid_ifc_generator.py
    
    Args:
        ifc: IFC file object
        body_context: Body geometric context
        owner_history: IfcOwnerHistory entity
        row: CSV row dict
        
    Returns:
        IfcWall entity or None if invalid
    """
    try:
        # Parse CSV data
        min_x = float(row['min_x'])
        min_y = float(row['min_y'])
        min_z = float(row['min_z'])
        max_x = float(row['max_x'])
        max_y = float(row['max_y'])
        max_z = float(row['max_z'])
        dx = float(row['dx'])
        dy = float(row['dy'])
        dz = float(row['dz'])
        wall_id_num = row['wall_id']
        
        min_bound = [min_x, min_y, min_z]
        max_bound = [max_x, max_y, max_z]
        
        # Create wall geometry
        shape = create_box_geometry(ifc, body_context, min_bound, max_bound)
        
        if shape is None:
            print(f"[WARN] Failed to create geometry for wall {wall_id_num}, skipping")
            return None
        
        # Create IfcWall entity
        wall_entity = ifc.create_entity(
            "IfcWall",
            GlobalId=ifcopenshell.guid.new(),
            OwnerHistory=owner_history,
            Name=f"Wall_{wall_id_num}",
            Description=f"L={dx:.2f}m, W={dy:.2f}m, H={dz:.2f}m",
            Representation=shape,
            PredefinedType="SOLIDWALL"
        )
        
        return wall_entity
        
    except Exception as e:
        print(f"[ERROR] Failed to create wall from row {row.get('wall_id', '?')}: {e}")
        return None


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python bbox_obj_to_ifc_converter.py <input.csv> <output.ifc>")
        sys.exit(1)
    
    csv_path = Path(sys.argv[1])
    ifc_path = Path(sys.argv[2])
    csv_bboxes_to_ifc(csv_path, ifc_path)
