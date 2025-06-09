using System.IO;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;

[ExecuteAlways]
public class TestImporter : MonoBehaviour
{
	public string modelName;
	public float parentOffset = 0;
	public float objectOffset = 3;
	public float rotationY = 0;
	public float scale = 1;

	GameObject parent = null;
	bool shouldDestroyParent = false;
	void Update()
	{
		if (shouldDestroyParent)
		{
			shouldDestroyParent = false;

			if (Application.isPlaying)
				Destroy(parent);
			else
				DestroyImmediate(parent);
			parent = null;
		}

		if (modelName != null && modelName.Length > 0 && parent == null)
		{
			ImportQEMObjs(modelName);
		}
	}

	private void OnValidate()
	{
		if (parent != null)
		{
			shouldDestroyParent = true; // 삭제 예약
		}
	}

	public void ImportQEMObjs(string name)
	{
		string rootPath = "Assets/Resources/QEM/" + name;

		if (!Directory.Exists(rootPath))
		{
			return;
		}

		if (parent == null)
		{
			parent = new GameObject(name);
			parent.transform.position = new Vector3(0, 0, parentOffset);
		}

		string[] objFiles = Directory.GetFiles(rootPath, name + "_*.fbx");
		int objCount = 0;
		foreach (string objPath in objFiles)
		{
			string assetPath = objPath.Replace("\\", "/");
			string objName = Path.GetFileNameWithoutExtension(assetPath);
			if (parent.transform.Find(objName) != null)
			{
				continue;
			}

			// Import the OBJ asset if needed
			ModelImporter modelImporter = AssetImporter.GetAtPath(assetPath) as ModelImporter;
			if (modelImporter != null)
			{
				modelImporter.globalScale = 100;
				modelImporter.SaveAndReimport();
			}

			GameObject obj = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
			if (obj != null)
			{
				GameObject instance = (GameObject)PrefabUtility.InstantiatePrefab(obj);
				instance.name = objName;
				instance.transform.SetParent(parent.transform);
				instance.transform.localPosition = new Vector3(objectOffset * objCount, 0, 0);
				instance.transform.localRotation = Quaternion.Euler(0f, rotationY, 0f);
				instance.transform.localScale = new Vector3(scale, scale, scale);
				objCount++;
			}
			else
			{
				Debug.LogWarning($"Failed to load: {assetPath}");
			}
		}
	}
}
