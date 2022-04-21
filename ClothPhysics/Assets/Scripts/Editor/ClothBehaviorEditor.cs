using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using static ClothBehaviour;

[CustomEditor(typeof(ClothBehaviour))]

public class ClothBehaviorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        EditorGUILayout.HelpBox("Apply this modifier to any plane object to achieve realtime cloth phyisics behaviour." +
            " You can choose different preset parameter setups depending on the resolution of your mesh in the context menu.", MessageType.None);

        base.OnInspectorGUI();


        ClothBehaviour b = (ClothBehaviour)target;

        b.m_NodeMass.value = EditorGUILayout.Slider("Mass By Node", b.m_NodeMass.value, 0.0f, 1.0f);

        b.m_Gravity.value = EditorGUILayout.Vector3Field("Gravity", b.m_Gravity.value);

        b.m_NodeDamping.value = EditorGUILayout.Slider("Node Damping", b.m_NodeDamping.value, 0.0f, 5.0f);

        b.m_SpringDamping.value = EditorGUILayout.Slider("Node Damping", b.m_SpringDamping.value, 0.0f, 5.0f);

        b.m_TractionStiffness.value = EditorGUILayout.FloatField("Traction Stiffness", b.m_TractionStiffness.value);

        b.m_FlexionStiffness.value = EditorGUILayout.FloatField("Flexion Stiffness", b.m_FlexionStiffness.value);


        SerializedProperty fixersList = serializedObject.FindProperty("m_Fixers");
        SerializedProperty collidingSp = serializedObject.FindProperty("m_CollidingSpheres");
        SerializedProperty collidingPl = serializedObject.FindProperty("m_CollidingPlanes");


        b.m_AffectedByWind = EditorGUILayout.Toggle("Affected By Wind", b.m_AffectedByWind);

        if (b.m_AffectedByWind)
        {
            EditorGUILayout.HelpBox("Unity active Wind Zones in scene will be automatically added. Affecting wind force will be equal to the Wind Zones " +
                "resulting force, so by changing the parameters in these objects (Main, Frequency, Turbulence and Pulse Magnitude) the wind will change.",MessageType.Info);

            b.m_WindFriction = EditorGUILayout.Slider("Wind Friction", b.m_WindFriction, 0.0f, 1.0f);
            b.m_WindSolverPrecission = (WindPrecission)EditorGUILayout.EnumPopup("Wind Solver Precission", b.m_WindSolverPrecission);
        }


        b.m_FixingByTexture = EditorGUILayout.Toggle("Fixing By Gradient", b.m_FixingByTexture);

        if (b.m_FixingByTexture)

        {
            EditorGUILayout.HelpBox("In order to use the texture, enable read/write on its import settings and set the alpha source to grayscale.",MessageType.Info);
            b.m_Texture = (Texture2D)EditorGUILayout.ObjectField("Texture", b.m_Texture, typeof(Texture2D), false);

        }
        else
        {
            EditorGUILayout.PropertyField(fixersList);
        }

        b.m_CanCollide = EditorGUILayout.Toggle("Can Collide", b.m_CanCollide);

        if (b.m_CanCollide)
        {
            EditorGUILayout.HelpBox("Only plane and spheric objects will be accepted",MessageType.Info);
            b.m_PenaltyDamping = EditorGUILayout.Slider("Penalty Damping", b.m_PenaltyDamping,0f,100f);
            EditorGUILayout.PropertyField(collidingSp);
            EditorGUILayout.PropertyField(collidingPl);

        }

    }

}
