using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using static ClothBehaviour;

[CustomEditor(typeof(ClothBehaviour))]

public class ClothBehaviorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();



        ClothBehaviour b = (ClothBehaviour)target;

        b.m_NodeMass.value = EditorGUILayout.Slider("Mass By Node", b.m_NodeMass.value, 0.0f, 1.0f);

        b.m_Gravity.value = EditorGUILayout.Vector3Field("Gravity", b.m_Gravity.value);

        b.m_NodeDamping.value = EditorGUILayout.Slider("Node Damping", b.m_NodeDamping.value, 0.0f, 5.0f);

        b.m_SpringDamping.value = EditorGUILayout.Slider("Node Damping", b.m_SpringDamping.value, 0.0f, 5.0f);

        b.m_TractionStiffness.value = EditorGUILayout.FloatField("Traction Stiffness", b.m_TractionStiffness.value);

        b.m_FlexionStiffness.value = EditorGUILayout.FloatField("Flexion Stiffness", b.m_FlexionStiffness.value);


        SerializedProperty fixersList = serializedObject.FindProperty("m_Fixers");
        SerializedProperty collidingObjects = serializedObject.FindProperty("m_CollidingObjects");


        b.m_AffectedByWind = EditorGUILayout.Toggle("Affected By Wind", b.m_AffectedByWind);

        if (b.m_AffectedByWind)
        {
            b.m_WindFriction = EditorGUILayout.Slider("Wind Friction", b.m_WindFriction, 0.0f, 1.0f);
            b.m_WindSolverPrecission = (WindPrecission)EditorGUILayout.EnumPopup("Wind Solver Precission", b.m_WindSolverPrecission);
        }


        b.m_FixingByTexture = EditorGUILayout.Toggle("Fixing By Gradient", b.m_FixingByTexture);

        if (b.m_FixingByTexture)
        {
            b.m_Texture = (Texture2D)EditorGUILayout.ObjectField("Texture", b.m_Texture, typeof(Texture2D), false);

        }
        else
        {
            EditorGUILayout.PropertyField(fixersList);
        }

        b.m_CanCollide = EditorGUILayout.Toggle("Can Collide", b.m_CanCollide);

        if (b.m_CanCollide)
        {
            EditorGUILayout.PropertyField(collidingObjects);

        }

    }

}
