using UnityEditor;
using UnityEngine;

public class MapGeneratorWindow : EditorWindow
{
    private float cellSize          = 0.05f;
    private float wallHeight        = 2f;
    private float darknessThreshold = 0.4f;
    private int   blockSize         = 1;

    [MenuItem("Tools/Map Generator")]
    static void OpenWindow() => GetWindow<MapGeneratorWindow>("Map Generator");

    void OnGUI()
    {
        GUILayout.Label("3D wall generator", EditorStyles.boldLabel);

        cellSize          = EditorGUILayout.FloatField("Cell size", cellSize);
        wallHeight        = EditorGUILayout.FloatField("Wall height", wallHeight);
        darknessThreshold = EditorGUILayout.Slider("Darkness threshold", darknessThreshold, 0f, 1f);
        blockSize         = EditorGUILayout.IntField("Block size", blockSize);

        EditorGUILayout.Space();
        EditorGUILayout.HelpBox(
          "Select a .png map in Maps folder and click the button", MessageType.Info);

        if (GUILayout.Button("Generate for selected map"))
            GenerateForSelected();
    }

    void GenerateForSelected()
    {
        
        var oldBuilders = GameObject.FindObjectsOfType<MapWallBuilder>();
        foreach (var b in oldBuilders)
        {
            if (b.gameObject.name.StartsWith("MapBuilder_"))
            {
        #if UNITY_EDITOR
                Object.DestroyImmediate(b.gameObject);
        #else
                Object.Destroy(b.gameObject);
        #endif
            }
        }
        
        var selected = Selection.GetFiltered<Texture2D>(SelectionMode.Assets);
        if (selected.Length == 0)
        {
            EditorUtility.DisplayDialog("Map is not selected",
                "Select a map in Maps folder", "ok");
            return;
        }

        foreach (var map in selected)
            GenerateMapScene(map);
    }

    void GenerateMapScene(Texture2D map)
    {
        
        var floor = GameObject.Find("MapFloor");
        if (floor == null)
        {
            floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "MapFloor";
        }

        int W = map.width, H = map.height;
        float worldW = W * cellSize, worldH = H * cellSize;

        floor.transform.position = Vector3.zero;
        floor.transform.rotation = Quaternion.identity;
        floor.transform.localScale = new Vector3(worldW / 10f, 1f, worldH / 10f);

        var rend = floor.GetComponent<Renderer>();
        var mat = new Material(Shader.Find("Unlit/Texture"));
        mat.mainTexture = map;
        rend.sharedMaterial = mat;

        string goName = $"MapBuilder_{map.name}";
        var mbGO = GameObject.Find(goName) ?? new GameObject(goName);
        var mb   = mbGO.GetComponent<MapWallBuilder>() ?? mbGO.AddComponent<MapWallBuilder>();

        mb.mapTexture        = map;
        mb.cellSize          = cellSize;
        mb.wallHeight        = wallHeight;
        mb.darknessThreshold = darknessThreshold;
        mb.blockSize         = blockSize;

        mb.GenerateWallsInEditor();

        mb.enabled = false;

        mb.gameObject.transform.rotation = Quaternion.Euler(0f, 180f, 0f);

        Selection.activeGameObject = mbGO;
    }
}
