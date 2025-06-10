using UnityEngine;

public class MapWallBuilder : MonoBehaviour
{
    public Texture2D mapTex;
    public float res = 0.05f;
    public float height = 2f;
    public float darkThreshold = 0.4f;
    public int step = 1;
    public Color color = Color.gray;

    public void GenerateWalls()
    {
        var children = transform.childCount;
        for (int i = children - 1; i >= 0; i--)
        {
            #if UNITY_EDITOR
                        DestroyImmediate(transform.GetChild(i).gameObject);
            #else
                        Destroy(transform.GetChild(i).gameObject);
            #endif
        }

        if (!mapTex) return;

        int w = mapTex.width;
        int h = mapTex.height;
        float xOff = -w * res / 2f;
        float zOff = -h * res / 2f;

        for (int i = 0; i < w; i += step)
        {
            for (int j = 0; j < h; j += step)
            {
                bool foundDark = false;

                for (int dx = 0; dx < step && !foundDark; dx++)
                {
                    for (int dy = 0; dy < step; dy++)
                    {
                        int px = i + dx;
                        int py = j + dy;
                        if (px >= w || py >= h) continue;

                        var pixel = mapTex.GetPixel(px, py);
                        if (pixel.grayscale < darkThreshold)
                        {
                            foundDark = true;
                            break;
                        }
                    }
                }

                if (!foundDark) continue;

                float pxMid = (i + step / 2f) * res + xOff;
                float pzMid = (j + step / 2f) * res + zOff;
                Vector3 pos = new Vector3(pxMid, height / 2f, pzMid);
                Vector3 scl = new Vector3(step * res, height, step * res);

                GameObject block = GameObject.CreatePrimitive(PrimitiveType.Cube);
                block.transform.parent = transform;
                block.transform.localPosition = pos;
                block.transform.localScale = scl;
                block.GetComponent<Renderer>().material.color = color;
            }
        }
    }
}
