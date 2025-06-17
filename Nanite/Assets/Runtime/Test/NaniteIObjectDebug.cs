using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

[ExecuteAlways]
public class RotateObject : MonoBehaviour
{
	[SerializeField] bool rotate = true;
	[SerializeField] float rotationSpeed = 25f;

	[Header("Visualization")]
	[SerializeField] bool drawTriangleNormals = true;
	[SerializeField] bool drawVertNormals = true;
	[SerializeField] float normalLength = 0.1f;
	[SerializeField] Color normalColor = Color.magenta;
	[SerializeField] bool drawAABBs = true;
	[SerializeField, Range(0, 1)] float aabbAlpha = 0.15f;
	[SerializeField] uint boundsIndex = 0;

	List<(Bounds, Color)> AABBs;

	void Update()
	{
		if (rotate && Application.isPlaying)
		{
			transform.Rotate(Vector3.up, rotationSpeed * Time.deltaTime, Space.Self);
		}

		if (drawAABBs && AABBs == null)
		{
			string folder = transform.parent.name;
			string name = gameObject.name;
			loadAABBsFromText("QEM/" + folder + "/" + gameObject.name + "_metadata");
		}
	}
	private void OnDisable()
	{
		AABBs = null;
	}

	void OnDrawGizmos()
	{
		Gizmos.matrix = transform.localToWorldMatrix;

		if (drawAABBs && AABBs != null)
		{
			if (boundsIndex > 0 && boundsIndex <= AABBs.Count)
			{
				var (bounds, color) = AABBs[(int)boundsIndex - 1];
				Gizmos.color = color;
				Gizmos.DrawWireCube(bounds.center, bounds.size);
				Gizmos.color = new Color(color.r, color.g, color.b, aabbAlpha);
				Gizmos.DrawCube(bounds.center, bounds.size);
				return;
			}
			else
			{
				foreach (var (bounds, color) in AABBs)
				{
					Gizmos.color = color;
					Gizmos.DrawWireCube(bounds.center, bounds.size);
					Gizmos.color = new Color(color.r, color.g, color.b, aabbAlpha);
					Gizmos.DrawCube(bounds.center, bounds.size);
				}
			}
		}

		if (drawTriangleNormals)
		{
			MeshFilter mf = GetComponent<MeshFilter>();
			if (mf == null || mf.sharedMesh == null)
				return;

			Mesh mesh = mf.sharedMesh;
			Vector3[] vertices = mesh.vertices;
			Vector3[] normals = mesh.normals;
			int[] triangles = mesh.triangles;

			Gizmos.color = normalColor;

			for (int i = 0; i < triangles.Length; i += 3)
			{
				Vector3 v0 = vertices[triangles[i]];
				Vector3 v1 = vertices[triangles[i + 1]];
				Vector3 v2 = vertices[triangles[i + 2]];

				Vector3 center = (v0 + v1 + v2) / 3f;

				Vector3 edge1 = v1 - v0;
				Vector3 edge2 = v2 - v0;
				Vector3 triNormal = Vector3.Cross(edge1, edge2).normalized;
				triNormal = transform.TransformDirection(triNormal);

				Gizmos.DrawLine(center, center + triNormal * normalLength);
			}
		}

		if (drawVertNormals)
		{
			MeshFilter mf = GetComponent<MeshFilter>();
			if (mf == null || mf.sharedMesh == null)
				return;

			Mesh mesh = mf.sharedMesh;
			Vector3[] vertices = mesh.vertices;
			Vector3[] normals = mesh.normals;
			int[] triangles = mesh.triangles;

			Gizmos.color = normalColor;

			for (int i = 0; i < normals.Length; ++i)
			{
				Vector3 v = vertices[i];
				Vector3 n = normals[i];

				Gizmos.DrawLine(v, v + n * normalLength);
			}
		}
	}

		void loadAABBsFromText(string resourcePath)
	{
		Debug.Log(resourcePath);
		TextAsset textAsset = Resources.Load<TextAsset>(resourcePath);
		if (textAsset == null)
		{
			Debug.LogError("Failed to load AABB metadata from Resources.");
			return;
		}

		AABBs = new List<(Bounds, Color)>();
		string[] lines = textAsset.text.Split('\n');

		foreach (var line in lines)
		{
			if (string.IsNullOrWhiteSpace(line)) continue;

			string[] tokens = line.Trim().Split(' ');
			if (tokens.Length != 9) continue;

			float maxX = -float.Parse(tokens[0]);
			float minY = float.Parse(tokens[1]);
			float minZ = float.Parse(tokens[2]);
			float minX = -float.Parse(tokens[3]);
			float maxY = float.Parse(tokens[4]);
			float maxZ = float.Parse(tokens[5]);
			float r = float.Parse(tokens[6]);
			float g = float.Parse(tokens[7]);
			float b = float.Parse(tokens[8]);

			Vector3 min = new Vector3(minX, minY, minZ);
			Vector3 max = new Vector3(maxX, maxY, maxZ);
			AABBs.Add((new Bounds((min + max) * 0.5f, max - min), new Color(r, g, b)));
		}

		Debug.Log($"Loaded {AABBs.Count} AABBs.");
	}
}
