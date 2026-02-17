#!/usr/bin/env python3
"""
inspect_shapefile.py — Run this FIRST to understand your campus shapefile.

Usage:
  python3 inspect_shapefile.py /path/to/campus.shp

This will show you:
  - Coordinate system (CRS)
  - Feature count and geometry types
  - Attribute columns and their values
  - Polygon stats (vertex counts)
  - Suggested filters for the path planner
"""

import sys
import geopandas as gpd
from shapely.geometry import Polygon, MultiPolygon


def inspect(shp_path: str):
    print(f"\n{'='*60}")
    print(f"  SHAPEFILE INSPECTOR")
    print(f"  {shp_path}")
    print(f"{'='*60}\n")

    gdf = gpd.read_file(shp_path)

    print(f"CRS: {gdf.crs}")
    print(f"  Geographic (lat/lon): {gdf.crs.is_geographic if gdf.crs else '?'}")
    print(f"  Projected (meters):   {gdf.crs.is_projected if gdf.crs else '?'}")
    print(f"  EPSG:                 {gdf.crs.to_epsg() if gdf.crs else '?'}")
    print(f"\nFeatures: {len(gdf)}")
    print(f"Bounds:   {gdf.total_bounds}\n")

    geom_types = gdf.geometry.geom_type.value_counts()
    print("Geometry types:")
    for gtype, count in geom_types.items():
        print(f"  {gtype}: {count}")

    print(f"\nColumns: {list(gdf.columns)}\n")

    for col in gdf.columns:
        if col == 'geometry':
            continue
        unique = gdf[col].unique()
        print(f"  {col} ({len(unique)} unique): {list(unique[:10])}")
        if len(unique) > 10:
            print(f"    ... +{len(unique)-10} more")

    poly_count = 0
    verts = []
    for geom in gdf.geometry:
        if geom is None:
            continue
        polys = []
        if isinstance(geom, Polygon):
            polys = [geom]
        elif isinstance(geom, MultiPolygon):
            polys = list(geom.geoms)
        for p in polys:
            poly_count += 1
            verts.append(len(p.exterior.coords))

    if verts:
        print(f"\nPolygon stats:")
        print(f"  Count: {poly_count}")
        print(f"  Verts: min={min(verts)} max={max(verts)} avg={sum(verts)/len(verts):.0f}")
        print(f"  Total vertices: {sum(verts)}")

    print(f"\n{'='*60}")
    print("  SUGGESTIONS")
    print(f"{'='*60}\n")

    if poly_count > 0:
        print(f"✓ {poly_count} polygons found — usable as obstacles")
    else:
        print("✗ No polygons — may need a different shapefile layer")

    for col in gdf.columns:
        if col == 'geometry':
            continue
        unique = gdf[col].unique()
        if 2 <= len(unique) <= 20:
            print(f"\n  Filter candidate: '{col}'")
            print(f"  Values: {list(unique[:15])}")
            print(f"  Example: \"{col} == '{unique[0]}'\"")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 inspect_shapefile.py <path_to.shp>")
        sys.exit(1)
    inspect(sys.argv[1])