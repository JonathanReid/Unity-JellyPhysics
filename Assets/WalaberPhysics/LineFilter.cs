using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class LineFilter
{
    public static List<float> NormalisedPoints;
    public static List<float> ComparativePoints = new List<float>{-4.99681f, -6.124712f, -8.773441f, -12.88361f, -13.93748f, -16.58714f, -15.02915f, -13.01626f, -10.71067f, -7.323703f, -6.061004f, -4.484148f, -3.578561f, -2.342345f, -2.47243f, -3.339765f, -2.548833f, -1.077563f, 0.9387863f, 0.1235407f, -0.5381398f, -1.838986f, -2.485616f, -4.771467f, -9.129723f, -32.93124f, -13.13552f, -8.084856f, -9.172647f, -8.574255f, -6.370799f, -6.739998f, -5.388016f, -6.69284f, -5.869924f, -6.800889f, -6.083313f, -6.838363f, -7.205084f, -6.8053f, -0.04423466f};
    private static List<Vector2> _points;

    public static List<Vector2> Filter(List<Vector2> points)
    {
        Vector2[] temp = new Vector2[points.Count];
        points.CopyTo(temp);
        _points = new List<Vector2>(temp);

        //now we have a connected shape, lets remove any internal loops that may have been created.
        FilterInternalCrossPoints();

        return _points;
    }

    private static void RemoveEndPoints()
    {
        _points.RemoveAt(0);
        _points.RemoveAt(_points.Count - 1);

        _points.Add(_points[0]);
    }
	
    //Extends the start and end _points in case you've drawn a shape that doesnt connect up.
    private static bool AddExtensionsToEnds()
    {
        Vector2 dir = GetDir(0);
        _points.Insert(0, _points[0] + dir * 10);

        dir = GetDir(_points.Count - 1);
        _points.Add(_points[_points.Count - 1] + dir * 50);

        return true;
    }

    private static Vector2 GetDir(int startIndex)
    {
        Vector2 dir = _points[startIndex];
        int i = startIndex == 0 ? 1 : _points.Count - 2;
        int am = startIndex == 0 ? 1 : -1;
        while (_points[i] == dir)
        {
            i += am;
            if (i >= _points.Count || i < 0)
            {
                i = Mathf.Clamp(i, 0, _points.Count - 1);
                break;
            }
        }
        dir -= _points[i];
        dir.Normalize();
        return dir;
    }

    private static void ConvertEndPointsIntoBezier()
    {
        Vector2 startPoint = _points[0];
        Vector2 endPoint = _points[_points.Count - 2];

        Vector2 controlPoint1 = startPoint;
        Vector2 dir = GetDir(0);
        controlPoint1 += dir * (Vector3.Distance(startPoint, endPoint));

        Vector2 controlPoint2 = endPoint;
        controlPoint2 += GetDir(_points.Count - 2) * (Vector3.Distance(startPoint, endPoint));

        _points.RemoveAt(_points.Count - 1);
        _points.RemoveAt(0);

        int i = 0, l = 3;
        for (; i < l; ++i)
        {
            _points.Insert(0, (CalculateBezierPoint(i / (float)l, startPoint, controlPoint1, controlPoint2, endPoint)));
        }
        _points.Add(_points[0]);

    }
    //end point, control point, control point, end point
    private static Vector3 CalculateBezierPoint(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        float uuu = uu * u;
        float ttt = tt * t;

        Vector3 p = uuu * p0; //first term
        p += 3 * uu * t * p1; //second term
        p += 3 * u * tt * p2; //third term
        p += ttt * p3; //fourth term

        return p;
    }
    //looks to see if the stat and end _points cross, and finds the point they intersect and makes that the new start and end point.
    //returns false if there are no crossed _points, e.g an unclosed and unusable shape.
    private static bool FilterCrossPointsForEnds()
    {
        int i = 0, l = _points.Count - 1;
        Vector2 cross = Vector2.zero;
        bool crossedPoints = false;
        for (; i < l; ++i)
        {
            int j = _points.Count - 1, k = 0;
            for (; j > k; --j)
            {
                if (j > (i + 3) && LineIntersection(_points[i], _points[i + 1], _points[j], _points[j - 1], ref cross))
                {
                    _points.RemoveRange(j, _points.Count - j);
                    _points.RemoveRange(0, i + 1);
                    crossedPoints = true;
                    goto OutLoop;
                }
            }
        }
        goto EndLoop;

        OutLoop:
        {
            _points.Insert(0, cross);
            _points.Add(cross);
        }

        EndLoop:
        {
            if (!crossedPoints)
            {
                FilterInternalCrossPoints();
            }
        }
        return crossedPoints;
    }
    //Looks through all internal lines, trying to find and internal loops or crossing _points, if it finds them it removes them.
    private static void FilterInternalCrossPoints()
    {
        bool crossPointFound = true;

        while (crossPointFound)
        {
            crossPointFound = false;
            int i = 1, l = _points.Count - 1;
            Vector2 cross = Vector2.zero;

            for (; i < l; ++i)
            {
                int j = _points.Count - 2, k = 0;
                for (; j > k; --j)
                {
                    if (j > i + 2 && LineIntersection(_points[i], _points[i + 1], _points[j], _points[j - 1], ref cross))
                    {
                        _points.RemoveRange(i, (j - i));
                        _points.Insert(i, cross);
                        crossPointFound = true;
                        goto OutLoop;
                    }
                }
            }
            OutLoop:
            {
            }
        }
    }

    public static bool LineIntersection(Vector2 linePoint1, Vector2 linePoint2, Vector2 linePoint3, Vector2 linePoint4, ref Vector2 intersection)
    {
        Vector2 a = linePoint2 - linePoint1;
        Vector2 b = linePoint3 - linePoint4;
        Vector2 c = linePoint1 - linePoint3;

        float alphaNumerator = b.y * c.x - b.x * c.y;
        float alphaDenominator = a.y * b.x - a.x * b.y;
        float betaNumerator = a.x * c.y - a.y * c.x;
        float betaDenominator = alphaDenominator;

        bool doIntersect = true;

        if (alphaDenominator == 0 || betaDenominator == 0)
        {
            doIntersect = false;
        }
        else
        {

            if (alphaDenominator > 0)
            {
                if (alphaNumerator < 0 || alphaNumerator > alphaDenominator)
                {
                    doIntersect = false;
                }
            }
            else if (alphaNumerator > 0 || alphaNumerator < alphaDenominator)
            {
                doIntersect = false;
            }

            if (doIntersect && betaDenominator > 0)
            {
                if (betaNumerator < 0 || betaNumerator > betaDenominator)
                {
                    doIntersect = false;
                }
            }
            else if (betaNumerator > 0 || betaNumerator < betaDenominator)
            {
                doIntersect = false;
            }
        }

        float Ax, Ay, f, num;

        Ax = linePoint2.x - linePoint1.x;
        Ay = linePoint2.y - linePoint1.y;

        num = alphaNumerator * Ax; // numerator //
        f = alphaDenominator;
        intersection.x = linePoint1.x + num / f;

        num = alphaNumerator * Ay;
        intersection.y = linePoint1.y + num / f;
        return doIntersect;
    }
}
