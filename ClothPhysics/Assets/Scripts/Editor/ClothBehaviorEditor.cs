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

        b.m_NodeMass.value = EditorGUILayout.Slider(new GUIContent("Mass By Node", "Controls the mass each vertex weights, not the entire mesh."), b.m_NodeMass.value, 0.0f, 1.0f);

        b.m_Gravity.value = EditorGUILayout.Vector3Field("Gravity", b.m_Gravity.value);

        b.m_NodeDamping.value = EditorGUILayout.Slider(new GUIContent("Node Damping", "A reduction in the amplitude of an oscillation as a result of energy being drained from the system to overcome frictional or other resistive forces. Higher values means more reduction in vertex movement."), b.m_NodeDamping.value, 0.0f, 5.0f);

        b.m_SpringDamping.value = EditorGUILayout.Slider(new GUIContent("Spring Damping", "A reduction in the amplitude of an oscillation as a result of energy being drained from the system to overcome frictional or other resistive forces. Higher values means more reduction in spring contraction forces."), b.m_SpringDamping.value, 0.0f, 5.0f);

        b.m_TractionStiffness.value = EditorGUILayout.FloatField(new GUIContent("Traction Stiffness","Controls the stiffness of the traction springs.These springs control horizontal and vertical movement. The more stiff, the less the mesh will deform and the quicker it will return to initial state."), b.m_TractionStiffness.value);

        b.m_FlexionStiffness.value = EditorGUILayout.FloatField(new GUIContent("Flexion Stiffness","Controls the stiffness of the flexion springs. These springs control shearing and diagonal movement. The more stiff, the less the mesh will deform and the quicker it will return to initial state."), b.m_FlexionStiffness.value);

        

        SerializedProperty fixersList = serializedObject.FindProperty("m_Fixers");
        SerializedProperty collidingSp = serializedObject.FindProperty("m_CollidingMeshes");


        b.m_AffectedByWind = EditorGUILayout.Toggle("Affected By Wind", b.m_AffectedByWind);

        if (b.m_AffectedByWind)
        {
            EditorGUILayout.HelpBox("Unity active Wind Zones in scene will be automatically added. Affecting wind force will be equal to the Wind Zones " +
                "resulting force, so by changing the parameters in these objects (Main, Frequency, Turbulence and Pulse Magnitude) the wind will change.",MessageType.Info);

            b.m_WindFriction = EditorGUILayout.Slider(new GUIContent("Wind Friction","Friction applied by the wind to the mesh surface. Higher values means more resistance."), b.m_WindFriction, 0.0f, 1.0f);
            b.m_WindSolverPrecission = (WindPrecission)EditorGUILayout.EnumPopup(new GUIContent("Wind Solver Precission", "Controls how many iterations is the wind force computed. Higher values means more computational load."), b.m_WindSolverPrecission);
        }


        b.m_FixingByTexture = EditorGUILayout.Toggle(new GUIContent("Fixing By Gradient","Choose between fixing the mesh to other game objects or using a grayscale texture to fix the nodes to the whiter values. The darker the value the less the node will be affected by the cloth behaviour."), b.m_FixingByTexture);

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
            EditorGUILayout.HelpBox("Only plane and spheric objects will behave in a proper way. Cubic ones are still being worked on.",MessageType.Info);
            b.m_PenaltyStiffness = EditorGUILayout.Slider(new GUIContent("Penalty Stiffness","Controls the power of the penalty force."), b.m_PenaltyStiffness,0f,100f);
            b.m_CollisionOffsetDistance= EditorGUILayout.Slider(new GUIContent("Collision Offset","Controls at how much distance from the surface of the collider this force starts to be applied to the mesh. Higher values useful on low res meshes."), b.m_CollisionOffsetDistance,0f,5f);
            EditorGUILayout.PropertyField(collidingSp);

        }

    }

}
