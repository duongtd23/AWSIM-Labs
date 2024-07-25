using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public static class BoundingBoxUtils
    {
        public static Rect Intersects(this Rect r1, Rect r2)
        {
            Rect area = new Rect();
            if (!r2.Overlaps(r1))
            {
                Debug.LogError("[AWAnalysis] The two rectangles are not overlap");
                return area;
            }
            float x1 = Mathf.Min(r1.xMax, r2.xMax);
            float x2 = Mathf.Max(r1.xMin, r2.xMin);
            float y1 = Mathf.Min(r1.yMax, r2.yMax);
            float y2 = Mathf.Max(r1.yMin, r2.yMin);
            area.x = Mathf.Min(x1, x2);
            area.y = Mathf.Min(y1, y2);
            area.width = Mathf.Max(0.0f, x1 - x2);
            area.height = Mathf.Max(0.0f, y1 - y2);
            return area;
        }

        /// <summary>
        /// Return the ratio between the overlap of r1 & r2 and
        /// the square of r2.
        /// </summary>
        /// <param name="r1"> Imagine r1 as the detected bounding box (by perception module)</param>
        /// <param name="r2"> The ground-trust bounding box of an agent concerned</param>
        /// <returns></returns>
        public static float OverlapRatio(Rect r1, Rect r2)
        {
            if (!r2.Overlaps(r1))
                return 0;
            Rect interection = Intersects(r1, r2);
            return interection.width * interection.height /
                (r2.width * r2.height);
        }

        /// <summary>
        /// Return the ratio between the false recognized region and
        /// the square of r1
        /// </summary>
        /// <param name="r1"></param>
        /// <param name="r2"></param>
        /// <returns></returns>
        public static float FalseRatio(Rect r1, Rect r2)
        {
            if (!r2.Overlaps(r1))
                return 100;
            Rect interection = Intersects(r1, r2);
            float falseRegion = r1.width * r1.height - interection.width * interection.height;
            return falseRegion /
                (r1.width * r1.height);
        }

        /// <summary>
        /// return the ratio between the intersection (overlap) of $r1 and $r2 and
        /// the union square of r1 and r2
        /// </summary>
        /// <param name="r1"> Imagine r1 as the detected bounding box (by perception module)</param>
        /// <param name="r2"> The ground-trust bounding box of an agent concerned</param>
        /// <returns></returns>
        public static float IOURatio(Rect r1, Rect r2)
        {
            if (!r2.Overlaps(r1))
                return 0;
            Rect interection = Intersects(r1, r2);
            float falseRegion = r1.width * r1.height - interection.width * interection.height;
            return interection.width * interection.height /
                (r2.width * r2.height + falseRegion);
        }
    }
}