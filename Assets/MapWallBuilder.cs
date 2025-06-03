using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class MapWallBuilder : MonoBehaviour
{
    [Tooltip("Texture")]
    public Texture2D mapTexture;

    [Tooltip("resolution in .yaml file")]
    public float cellSize = 0.05f;

    [Tooltip("Height of walls")]
    public float wallHeight = 2f;

    [Range(0f, 1f), Tooltip("Darkness threshold")]
    public float darknessThreshold = 0.4f;

    [Tooltip("Group size")]
    public int blockSize = 1;

    [Tooltip("Wall color")]
    public Color wallColor = Color.gray;

    void Start()
    {
        if (mapTexture == null)
        {
            Debug.LogError("Assign mapTexture");
            return;
        }

        int W = mapTexture.width;
        int H = mapTexture.height;
        float worldWidth  = W * cellSize;
        float worldHeight = H * cellSize;

        var floor = GameObject.Find("MapFloor");
        if (floor != null)
        {
            floor.transform.localScale = new Vector3(worldWidth / 10f, 1f, worldHeight / 10f);
            floor.transform.position   = Vector3.zero;
            floor.transform.rotation   = Quaternion.identity;
        }
        else
        {
            Debug.LogWarning("GameObject not found");
        }

        float originOffsetX = -worldWidth  * 0.5f;
        float originOffsetZ = -worldHeight * 0.5f;
        int placed = 0;
        for (int bx = 0; bx < W; bx += blockSize)
        {
            for (int by = 0; by < H; by += blockSize)
            {
                bool isObstacle = false;
                int bw = Mathf.Min(blockSize, W - bx);
                int bh = Mathf.Min(blockSize, H - by);

                for (int dx = 0; dx < bw && !isObstacle; dx++)
                {
                    for (int dy = 0; dy < bh; dy++)
                    {
                        if (mapTexture.GetPixel(bx + dx, by + dy).grayscale < darknessThreshold)
                        {
                            isObstacle = true;
                            break;
                        }
                    }
                }

                if (!isObstacle) continue;

                float localX = (bx + bw * 0.5f) * cellSize;
                float localZ = (by + bh * 0.5f) * cellSize;
                float worldX = originOffsetX + localX;
                float worldZ = originOffsetZ + localZ;

                Vector3 pos   = new Vector3(worldX, wallHeight * 0.5f, worldZ);
                Vector3 scale = new Vector3(bw * cellSize, wallHeight, bh * cellSize);

                var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                cube.transform.parent       = transform;
                cube.transform.localPosition = pos;
                cube.transform.localScale    = scale;

                var rend = cube.GetComponent<Renderer>();
                rend.material.color = wallColor;

                placed++;
            }
        }
    }

    void OnDisable()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            var child = transform.GetChild(i).gameObject;
            #if UNITY_EDITOR
            DestroyImmediate(child);
            #else
            Destroy(child);
            #endif
        }
    }

    #if UNITY_EDITOR
    public void GenerateWallsInEditor()
    {
        OnDisable();
        Start();
    }
    #endif

}
