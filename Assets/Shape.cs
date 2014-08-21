using UnityEngine;
using System.Collections;
using Poly2Tri;

public class Shape : MonoBehaviour{

	[SerializeField]
	public GameObject BuiltGameObject;
	[SerializeField]
    public Vector2 BoundingBox;
	[SerializeField]
    public Vector2 UVBounds;
	[SerializeField]
    public float Area;
	[SerializeField]
    public Vector2[] Points;
	[SerializeField]
    public Polygon Polygon;
}
