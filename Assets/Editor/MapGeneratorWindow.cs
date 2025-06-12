using UnityEditor;
using UnityEngine;

public class MapGeneratorWindow : EditorWindow
{
    float resolution = 0.05f;
    float wallH = 2f;
    float threshold = 0.4f;
    int blockSz = 1;

    [MenuItem("Tools/Map generator")]
    static void ShowWindow()
    {
        GetWindow<MapGeneratorWindow>("Map generator");
    }

    void OnGUI()
    {
        GUILayout.Label("Settings", EditorStyles.boldLabel);

        // Input fields
        resolution = EditorGUILayout.FloatField("resolution from yaml", resolution);
        wallH = EditorGUILayout.FloatField("wall height", wallH);
        threshold = EditorGUILayout.Slider("darkness threshold", threshold, 0f, 1f);
        blockSz = EditorGUILayout.IntField("group size", blockSz);

        GUILayout.Space(5);
        EditorGUILayout.HelpBox("select png map from Maps folder", MessageType.Info);

        if (GUILayout.Button("Generate"))
        {
            GenWalls();
        }
    }

    void GenWalls()
    {
        var existing = GameObject.FindObjectsOfType<MapWallBuilder>();
        foreach (var e in existing)
        {
            if (e.name.StartsWith("MapBuilder_"))
            {
#if UNITY_EDITOR
                DestroyImmediate(e.gameObject);
#else
                Destroy(e.gameObject);
#endif
            }
        }

        var maps = Selection.GetFiltered<Texture2D>(SelectionMode.Assets);
        if (maps == null || maps.Length == 0)
        {
            EditorUtility.DisplayDialog("no map", "select a map", "ok");
            return;
        }

        foreach (var m in maps)
        {
            CreateScene(m);
        }
    }

    void CreateScene(Texture2D m)
    {
        var floor = GameObject.Find("MapFloor");
        if (!floor)
        {
            floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "MapFloor";
        }

        float fw = m.width * resolution;
        float fh = m.height * resolution;

        floor.transform.localScale = new Vector3(fw / 10f, 1, fh / 10f);
        floor.transform.position = Vector3.zero;

        var r = floor.GetComponent<Renderer>();
        var mat = new Material(Shader.Find("Unlit/Texture"));
        mat.mainTexture = m;
        r.sharedMaterial = mat;

        var mapGOName = "MapBuilder_" + m.name;
        var go = GameObject.Find(mapGOName);
        if (!go) go = new GameObject(mapGOName);

        var builder = go.GetComponent<MapWallBuilder>();
        if (!builder) builder = go.AddComponent<MapWallBuilder>();

        builder.mapTex = m;
        builder.res = resolution;
        builder.height = wallH;
        builder.darkThreshold = threshold;
        builder.step = blockSz;

        builder.GenerateWalls();
        builder.enabled = false;

        go.transform.rotation = Quaternion.Euler(0, 180, 0);
        Selection.activeGameObject = go;
    }
}
