using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(ClothBehaviour))]

public class ClothBehaviorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        ClothBehaviour b = (ClothBehaviour)target;


        SerializedProperty windFriction = serializedObject.FindProperty("m_WindFriction");
        SerializedProperty fixersList = serializedObject.FindProperty("m_Fixers");
        SerializedProperty penaltyPrecission = serializedObject.FindProperty("m_PenaltyForcePrecission");
        SerializedProperty collidingObjects = serializedObject.FindProperty("m_CollidingObjects");

        b.m_AffectedByWind = EditorGUILayout.Toggle("Affected By Wind", b.m_AffectedByWind);

        if (b.m_AffectedByWind)
        {
            EditorGUILayout.PropertyField(windFriction);
        }


        b.m_FixingByTexture = EditorGUILayout.Toggle("Fixing By Gradient", b.m_FixingByTexture);

        if (b.m_FixingByTexture)
        {
            b.m_Texture = (Texture2D)EditorGUILayout.ObjectField("Texture",b.m_Texture, typeof(Texture2D), false);

        }
        else
        {
            EditorGUILayout.PropertyField(fixersList);
        }
        
        b.m_CanCollide = EditorGUILayout.Toggle("Can Collide", b.m_CanCollide);

        if (b.m_CanCollide)
        {
            EditorGUILayout.PropertyField(penaltyPrecission);
            EditorGUILayout.PropertyField(collidingObjects);

        }

    }
}
