from pathlib import Path
from typing import List, Optional, Dict
from pydantic_xml import attr, BaseXmlModel
from pydantic import PrivateAttr


class Mujoco(BaseXmlModel, tag="mujoco", search_mode="unordered"):
    _filename: Path = PrivateAttr(default_factory=Path)
    _joint_order: Dict[str, List[str]] = PrivateAttr(default_factory=dict)
    _tendon_order: Dict[str, List[str]] = PrivateAttr(default_factory=dict)
    model_: str = attr("model", default=None)

    class Compiler(BaseXmlModel, tag="compiler", search_mode="unordered"):
        angle_: str = attr("angle", default=None)
        assetdir_: str = attr("assetdir", default=None)
        autolimits_: str = attr("autolimits", default=None)
        balanceinertia_: str = attr("balanceinertia", default=None)
        boundinertia_: str = attr("boundinertia", default=None)
        boundmass_: str = attr("boundmass", default=None)
        convexhull_: str = attr("convexhull", default=None)
        coordinate_: str = attr("coordinate", default=None)
        discardvisual_: str = attr("discardvisual", default=None)
        eulerseq_: str = attr("eulerseq", default=None)
        exactmeshinertia_: str = attr("exactmeshinertia", default=None)
        fitaabb_: str = attr("fitaabb", default=None)
        fusestatic_: str = attr("fusestatic", default=None)
        inertiafromgeom_: str = attr("inertiafromgeom", default=None)
        inertiagrouprange_: str = attr("inertiagrouprange", default=None)
        meshdir_: str = attr("meshdir", default=None)
        settotalmass_: str = attr("settotalmass", default=None)
        strippath_: str = attr("strippath", default=None)
        texturedir_: str = attr("texturedir", default=None)
        usethread_: str = attr("usethread", default=None)

        class Lengthrange(BaseXmlModel, tag="lengthrange", search_mode="unordered"):
            accel_: str = attr("accel", default=None)
            interval_: str = attr("interval", default=None)
            inttotal_: str = attr("inttotal", default=None)
            maxforce_: str = attr("maxforce", default=None)
            mode_: str = attr("mode", default=None)
            timeconst_: str = attr("timeconst", default=None)
            timestep_: str = attr("timestep", default=None)
            tolrange_: str = attr("tolrange", default=None)
            useexisting_: str = attr("useexisting", default=None)
            uselimit_: str = attr("uselimit", default=None)

        lengthrange_: Optional[Lengthrange] = None

    class Option(BaseXmlModel, tag="option", search_mode="unordered"):
        actuatorgroupdisable_: str = attr("actuatorgroupdisable", default=None)
        apirate_: str = attr("apirate", default=None)
        cone_: str = attr("cone", default=None)
        density_: str = attr("density", default=None)
        gravity_: str = attr("gravity", default=None)
        impratio_: str = attr("impratio", default=None)
        integrator_: str = attr("integrator", default=None)
        iterations_: str = attr("iterations", default=None)
        jacobian_: str = attr("jacobian", default=None)
        ls_iterations_: str = attr("ls_iterations", default=None)
        ls_tolerance_: str = attr("ls_tolerance", default=None)
        magnetic_: str = attr("magnetic", default=None)
        mpr_iterations_: str = attr("mpr_iterations", default=None)
        mpr_tolerance_: str = attr("mpr_tolerance", default=None)
        noslip_iterations_: str = attr("noslip_iterations", default=None)
        noslip_tolerance_: str = attr("noslip_tolerance", default=None)
        o_friction_: str = attr("o_friction", default=None)
        o_margin_: str = attr("o_margin", default=None)
        o_solimp_: str = attr("o_solimp", default=None)
        o_solref_: str = attr("o_solref", default=None)
        sdf_initpoints_: str = attr("sdf_initpoints", default=None)
        sdf_iterations_: str = attr("sdf_iterations", default=None)
        solver_: str = attr("solver", default=None)
        timestep_: str = attr("timestep", default=None)
        tolerance_: str = attr("tolerance", default=None)
        viscosity_: str = attr("viscosity", default=None)
        wind_: str = attr("wind", default=None)

        class Flag(BaseXmlModel, tag="flag", search_mode="unordered"):
            actuation_: str = attr("actuation", default=None)
            autoreset_: str = attr("autoreset", default=None)
            clampctrl_: str = attr("clampctrl", default=None)
            constraint_: str = attr("constraint", default=None)
            contact_: str = attr("contact", default=None)
            energy_: str = attr("energy", default=None)
            equality_: str = attr("equality", default=None)
            eulerdamp_: str = attr("eulerdamp", default=None)
            filterparent_: str = attr("filterparent", default=None)
            frictionloss_: str = attr("frictionloss", default=None)
            fwdinv_: str = attr("fwdinv", default=None)
            gravity_: str = attr("gravity", default=None)
            invdiscrete_: str = attr("invdiscrete", default=None)
            island_: str = attr("island", default=None)
            limit_: str = attr("limit", default=None)
            midphase_: str = attr("midphase", default=None)
            multiccd_: str = attr("multiccd", default=None)
            override_: str = attr("override", default=None)
            passive_: str = attr("passive", default=None)
            refsafe_: str = attr("refsafe", default=None)
            sensor_: str = attr("sensor", default=None)
            warmstart_: str = attr("warmstart", default=None)

        flag_: Optional[Flag] = None

    class Size(BaseXmlModel, tag="size", search_mode="unordered"):
        memory_: str = attr("memory", default=None)
        nconmax_: str = attr("nconmax", default=None)
        njmax_: str = attr("njmax", default=None)
        nkey_: str = attr("nkey", default=None)
        nstack_: str = attr("nstack", default=None)
        nuser_actuator_: str = attr("nuser_actuator", default=None)
        nuser_body_: str = attr("nuser_body", default=None)
        nuser_cam_: str = attr("nuser_cam", default=None)
        nuser_geom_: str = attr("nuser_geom", default=None)
        nuser_jnt_: str = attr("nuser_jnt", default=None)
        nuser_sensor_: str = attr("nuser_sensor", default=None)
        nuser_site_: str = attr("nuser_site", default=None)
        nuser_tendon_: str = attr("nuser_tendon", default=None)
        nuserdata_: str = attr("nuserdata", default=None)

    class Visual(BaseXmlModel, tag="visual", search_mode="unordered"):

        class Global(BaseXmlModel, tag="global", search_mode="unordered"):
            azimuth_: str = attr("azimuth", default=None)
            bvactive_: str = attr("bvactive", default=None)
            elevation_: str = attr("elevation", default=None)
            ellipsoidinertia_: str = attr("ellipsoidinertia", default=None)
            fovy_: str = attr("fovy", default=None)
            glow_: str = attr("glow", default=None)
            ipd_: str = attr("ipd", default=None)
            linewidth_: str = attr("linewidth", default=None)
            offheight_: str = attr("offheight", default=None)
            offwidth_: str = attr("offwidth", default=None)
            orthographic_: str = attr("orthographic", default=None)
            realtime_: str = attr("realtime", default=None)

        class Quality(BaseXmlModel, tag="quality", search_mode="unordered"):
            numquads_: str = attr("numquads", default=None)
            numslices_: str = attr("numslices", default=None)
            numstacks_: str = attr("numstacks", default=None)
            offsamples_: str = attr("offsamples", default=None)
            shadowsize_: str = attr("shadowsize", default=None)

        class Headlight(BaseXmlModel, tag="headlight", search_mode="unordered"):
            active_: str = attr("active", default=None)
            ambient_: str = attr("ambient", default=None)
            diffuse_: str = attr("diffuse", default=None)
            specular_: str = attr("specular", default=None)

        class Map(BaseXmlModel, tag="map", search_mode="unordered"):
            actuatortendon_: str = attr("actuatortendon", default=None)
            alpha_: str = attr("alpha", default=None)
            fogend_: str = attr("fogend", default=None)
            fogstart_: str = attr("fogstart", default=None)
            force_: str = attr("force", default=None)
            haze_: str = attr("haze", default=None)
            shadowclip_: str = attr("shadowclip", default=None)
            shadowscale_: str = attr("shadowscale", default=None)
            stiffness_: str = attr("stiffness", default=None)
            stiffnessrot_: str = attr("stiffnessrot", default=None)
            torque_: str = attr("torque", default=None)
            zfar_: str = attr("zfar", default=None)
            znear_: str = attr("znear", default=None)

        class Scale(BaseXmlModel, tag="scale", search_mode="unordered"):
            actuatorlength_: str = attr("actuatorlength", default=None)
            actuatorwidth_: str = attr("actuatorwidth", default=None)
            camera_: str = attr("camera", default=None)
            com_: str = attr("com", default=None)
            connect_: str = attr("connect", default=None)
            constraint_: str = attr("constraint", default=None)
            contactheight_: str = attr("contactheight", default=None)
            contactwidth_: str = attr("contactwidth", default=None)
            forcewidth_: str = attr("forcewidth", default=None)
            framelength_: str = attr("framelength", default=None)
            framewidth_: str = attr("framewidth", default=None)
            frustum_: str = attr("frustum", default=None)
            jointlength_: str = attr("jointlength", default=None)
            jointwidth_: str = attr("jointwidth", default=None)
            light_: str = attr("light", default=None)
            selectpoint_: str = attr("selectpoint", default=None)
            slidercrank_: str = attr("slidercrank", default=None)

        class Rgba(BaseXmlModel, tag="rgba", search_mode="unordered"):
            actuator_: str = attr("actuator", default=None)
            actuatornegative_: str = attr("actuatornegative", default=None)
            actuatorpositive_: str = attr("actuatorpositive", default=None)
            bv_: str = attr("bv", default=None)
            bvactive_: str = attr("bvactive", default=None)
            camera_: str = attr("camera", default=None)
            com_: str = attr("com", default=None)
            connect_: str = attr("connect", default=None)
            constraint_: str = attr("constraint", default=None)
            contactforce_: str = attr("contactforce", default=None)
            contactfriction_: str = attr("contactfriction", default=None)
            contactgap_: str = attr("contactgap", default=None)
            contactpoint_: str = attr("contactpoint", default=None)
            contacttorque_: str = attr("contacttorque", default=None)
            crankbroken_: str = attr("crankbroken", default=None)
            fog_: str = attr("fog", default=None)
            force_: str = attr("force", default=None)
            frustum_: str = attr("frustum", default=None)
            haze_: str = attr("haze", default=None)
            inertia_: str = attr("inertia", default=None)
            joint_: str = attr("joint", default=None)
            light_: str = attr("light", default=None)
            rangefinder_: str = attr("rangefinder", default=None)
            selectpoint_: str = attr("selectpoint", default=None)
            slidercrank_: str = attr("slidercrank", default=None)

        global_: Optional[Global] = None
        quality_: Optional[Quality] = None
        headlight_: Optional[Headlight] = None
        map_: Optional[Map] = None
        scale_: Optional[Scale] = None
        rgba_: Optional[Rgba] = None

    class Statistic(BaseXmlModel, tag="statistic", search_mode="unordered"):
        center_: str = attr("center", default=None)
        extent_: str = attr("extent", default=None)
        meaninertia_: str = attr("meaninertia", default=None)
        meanmass_: str = attr("meanmass", default=None)
        meansize_: str = attr("meansize", default=None)

    class Default(BaseXmlModel, tag="default", search_mode="unordered"):
        class_: str = attr("class", default=None)

        class Mesh(BaseXmlModel, tag="mesh", search_mode="unordered"):
            maxhullvert_: str = attr("maxhullvert", default=None)
            scale_: str = attr("scale", default=None)

        class Material(BaseXmlModel, tag="material", search_mode="unordered"):
            emission_: str = attr("emission", default=None)
            metallic_: str = attr("metallic", default=None)
            reflectance_: str = attr("reflectance", default=None)
            rgba_: str = attr("rgba", default=None)
            roughness_: str = attr("roughness", default=None)
            shininess_: str = attr("shininess", default=None)
            specular_: str = attr("specular", default=None)
            texrepeat_: str = attr("texrepeat", default=None)
            texture_: str = attr("texture", default=None)
            texuniform_: str = attr("texuniform", default=None)

        class Joint(BaseXmlModel, tag="joint", search_mode="unordered"):
            actuatorfrclimited_: str = attr("actuatorfrclimited", default=None)
            actuatorfrcrange_: str = attr("actuatorfrcrange", default=None)
            actuatorgravcomp_: str = attr("actuatorgravcomp", default=None)
            armature_: str = attr("armature", default=None)
            axis_: str = attr("axis", default=None)
            damping_: str = attr("damping", default=None)
            frictionloss_: str = attr("frictionloss", default=None)
            group_: str = attr("group", default=None)
            limited_: str = attr("limited", default=None)
            margin_: str = attr("margin", default=None)
            pos_: str = attr("pos", default=None)
            range_: str = attr("range", default=None)
            ref_: str = attr("ref", default=None)
            solimpfriction_: str = attr("solimpfriction", default=None)
            solimplimit_: str = attr("solimplimit", default=None)
            solreffriction_: str = attr("solreffriction", default=None)
            solreflimit_: str = attr("solreflimit", default=None)
            springdamper_: str = attr("springdamper", default=None)
            springref_: str = attr("springref", default=None)
            stiffness_: str = attr("stiffness", default=None)
            type_: str = attr("type", default=None)
            user_: str = attr("user", default=None)

        class Geom(BaseXmlModel, tag="geom", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            conaffinity_: str = attr("conaffinity", default=None)
            condim_: str = attr("condim", default=None)
            contype_: str = attr("contype", default=None)
            density_: str = attr("density", default=None)
            euler_: str = attr("euler", default=None)
            fitscale_: str = attr("fitscale", default=None)
            fluidcoef_: str = attr("fluidcoef", default=None)
            fluidshape_: str = attr("fluidshape", default=None)
            friction_: str = attr("friction", default=None)
            fromto_: str = attr("fromto", default=None)
            gap_: str = attr("gap", default=None)
            group_: str = attr("group", default=None)
            hfield_: str = attr("hfield", default=None)
            margin_: str = attr("margin", default=None)
            mass_: str = attr("mass", default=None)
            material_: str = attr("material", default=None)
            mesh_: str = attr("mesh", default=None)
            pos_: str = attr("pos", default=None)
            priority_: str = attr("priority", default=None)
            quat_: str = attr("quat", default=None)
            rgba_: str = attr("rgba", default=None)
            shellinertia_: str = attr("shellinertia", default=None)
            size_: str = attr("size", default=None)
            solimp_: str = attr("solimp", default=None)
            solmix_: str = attr("solmix", default=None)
            solref_: str = attr("solref", default=None)
            type_: str = attr("type", default=None)
            user_: str = attr("user", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

        class Site(BaseXmlModel, tag="site", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            euler_: str = attr("euler", default=None)
            fromto_: str = attr("fromto", default=None)
            group_: str = attr("group", default=None)
            material_: str = attr("material", default=None)
            pos_: str = attr("pos", default=None)
            quat_: str = attr("quat", default=None)
            rgba_: str = attr("rgba", default=None)
            size_: str = attr("size", default=None)
            type_: str = attr("type", default=None)
            user_: str = attr("user", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

        class Camera(BaseXmlModel, tag="camera", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            euler_: str = attr("euler", default=None)
            focal_: str = attr("focal", default=None)
            focalpixel_: str = attr("focalpixel", default=None)
            fovy_: str = attr("fovy", default=None)
            ipd_: str = attr("ipd", default=None)
            mode_: str = attr("mode", default=None)
            orthographic_: str = attr("orthographic", default=None)
            pos_: str = attr("pos", default=None)
            principal_: str = attr("principal", default=None)
            principalpixel_: str = attr("principalpixel", default=None)
            quat_: str = attr("quat", default=None)
            resolution_: str = attr("resolution", default=None)
            sensorsize_: str = attr("sensorsize", default=None)
            user_: str = attr("user", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

        class Light(BaseXmlModel, tag="light", search_mode="unordered"):
            active_: str = attr("active", default=None)
            ambient_: str = attr("ambient", default=None)
            attenuation_: str = attr("attenuation", default=None)
            bulbradius_: str = attr("bulbradius", default=None)
            castshadow_: str = attr("castshadow", default=None)
            cutoff_: str = attr("cutoff", default=None)
            diffuse_: str = attr("diffuse", default=None)
            dir_: str = attr("dir", default=None)
            directional_: str = attr("directional", default=None)
            exponent_: str = attr("exponent", default=None)
            mode_: str = attr("mode", default=None)
            pos_: str = attr("pos", default=None)
            specular_: str = attr("specular", default=None)

        class Pair(BaseXmlModel, tag="pair", search_mode="unordered"):
            condim_: str = attr("condim", default=None)
            friction_: str = attr("friction", default=None)
            gap_: str = attr("gap", default=None)
            margin_: str = attr("margin", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)
            solreffriction_: str = attr("solreffriction", default=None)

        class Equality(BaseXmlModel, tag="equality", search_mode="unordered"):
            active_: str = attr("active", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)

        class Tendon(BaseXmlModel, tag="tendon", search_mode="unordered"):
            damping_: str = attr("damping", default=None)
            frictionloss_: str = attr("frictionloss", default=None)
            group_: str = attr("group", default=None)
            limited_: str = attr("limited", default=None)
            margin_: str = attr("margin", default=None)
            material_: str = attr("material", default=None)
            range_: str = attr("range", default=None)
            rgba_: str = attr("rgba", default=None)
            solimpfriction_: str = attr("solimpfriction", default=None)
            solimplimit_: str = attr("solimplimit", default=None)
            solreffriction_: str = attr("solreffriction", default=None)
            solreflimit_: str = attr("solreflimit", default=None)
            springlength_: str = attr("springlength", default=None)
            stiffness_: str = attr("stiffness", default=None)
            user_: str = attr("user", default=None)
            width_: str = attr("width", default=None)

        class General(BaseXmlModel, tag="general", search_mode="unordered"):
            actdim_: str = attr("actdim", default=None)
            actearly_: str = attr("actearly", default=None)
            actlimited_: str = attr("actlimited", default=None)
            actrange_: str = attr("actrange", default=None)
            biasprm_: str = attr("biasprm", default=None)
            biastype_: str = attr("biastype", default=None)
            cranklength_: str = attr("cranklength", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            dynprm_: str = attr("dynprm", default=None)
            dyntype_: str = attr("dyntype", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gainprm_: str = attr("gainprm", default=None)
            gaintype_: str = attr("gaintype", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            user_: str = attr("user", default=None)

        class Motor(BaseXmlModel, tag="motor", search_mode="unordered"):
            cranklength_: str = attr("cranklength", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            user_: str = attr("user", default=None)

        class Position(BaseXmlModel, tag="position", search_mode="unordered"):
            cranklength_: str = attr("cranklength", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            dampratio_: str = attr("dampratio", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            inheritrange_: str = attr("inheritrange", default=None)
            kp_: str = attr("kp", default=None)
            kv_: str = attr("kv", default=None)
            timeconst_: str = attr("timeconst", default=None)
            user_: str = attr("user", default=None)

        class Velocity(BaseXmlModel, tag="velocity", search_mode="unordered"):
            cranklength_: str = attr("cranklength", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            kv_: str = attr("kv", default=None)
            user_: str = attr("user", default=None)

        class Intvelocity(BaseXmlModel, tag="intvelocity", search_mode="unordered"):
            actrange_: str = attr("actrange", default=None)
            cranklength_: str = attr("cranklength", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            dampratio_: str = attr("dampratio", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            inheritrange_: str = attr("inheritrange", default=None)
            kp_: str = attr("kp", default=None)
            kv_: str = attr("kv", default=None)
            user_: str = attr("user", default=None)

        class Damper(BaseXmlModel, tag="damper", search_mode="unordered"):
            cranklength_: str = attr("cranklength", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            kv_: str = attr("kv", default=None)
            user_: str = attr("user", default=None)

        class Cylinder(BaseXmlModel, tag="cylinder", search_mode="unordered"):
            area_: str = attr("area", default=None)
            bias_: str = attr("bias", default=None)
            cranklength_: str = attr("cranklength", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            diameter_: str = attr("diameter", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            timeconst_: str = attr("timeconst", default=None)
            user_: str = attr("user", default=None)

        class Muscle(BaseXmlModel, tag="muscle", search_mode="unordered"):
            cranklength_: str = attr("cranklength", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            force_: str = attr("force", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            fpmax_: str = attr("fpmax", default=None)
            fvmax_: str = attr("fvmax", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            lmax_: str = attr("lmax", default=None)
            lmin_: str = attr("lmin", default=None)
            range_: str = attr("range", default=None)
            scale_: str = attr("scale", default=None)
            timeconst_: str = attr("timeconst", default=None)
            user_: str = attr("user", default=None)
            vmax_: str = attr("vmax", default=None)

        class Adhesion(BaseXmlModel, tag="adhesion", search_mode="unordered"):
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gain_: str = attr("gain", default=None)
            group_: str = attr("group", default=None)
            user_: str = attr("user", default=None)

        default_: List["Default"] = []

        mesh_: Optional[Mesh] = None
        material_: Optional[Material] = None
        joint_: Optional[Joint] = None
        geom_: Optional[Geom] = None
        site_: Optional[Site] = None
        camera_: Optional[Camera] = None
        light_: Optional[Light] = None
        pair_: Optional[Pair] = None
        equality_: Optional[Equality] = None
        tendon_: Optional[Tendon] = None
        general_: Optional[General] = None
        motor_: Optional[Motor] = None
        position_: Optional[Position] = None
        velocity_: Optional[Velocity] = None
        intvelocity_: Optional[Intvelocity] = None
        damper_: Optional[Damper] = None
        cylinder_: Optional[Cylinder] = None
        muscle_: Optional[Muscle] = None
        adhesion_: Optional[Adhesion] = None

    class Extension(BaseXmlModel, tag="extension", search_mode="unordered"):

        class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
            plugin_: str = attr("plugin", default=None)

            class Instance(BaseXmlModel, tag="instance", search_mode="unordered"):
                name_: str = attr("name", default=None)

                class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                    key_: str = attr("key", default=None)
                    value_: str = attr("value", default=None)

                config_: List[Config] = []

            instance_: List[Instance] = []

        plugin_: List[Plugin] = []

    class Custom(BaseXmlModel, tag="custom", search_mode="unordered"):

        class Numeric(BaseXmlModel, tag="numeric", search_mode="unordered"):
            data_: str = attr("data", default=None)
            name_: str = attr("name", default=None)
            size_: str = attr("size", default=None)

        class Text(BaseXmlModel, tag="text", search_mode="unordered"):
            data_: str = attr("data", default=None)
            name_: str = attr("name", default=None)

        class Tuple(BaseXmlModel, tag="tuple", search_mode="unordered"):
            name_: str = attr("name", default=None)

            class Element(BaseXmlModel, tag="element", search_mode="unordered"):
                objname_: str = attr("objname", default=None)
                objtype_: str = attr("objtype", default=None)
                prm_: str = attr("prm", default=None)

            element_: List[Element] = []

        numeric_: List[Numeric] = []
        text_: List[Text] = []
        tuple_: List[Tuple] = []

    class Asset(BaseXmlModel, tag="asset", search_mode="unordered"):

        class Mesh(BaseXmlModel, tag="mesh", search_mode="unordered"):
            class_: str = attr("class", default=None)
            content_type_: str = attr("content_type", default=None)
            face_: str = attr("face", default=None)
            file_: str = attr("file", default=None)
            maxhullvert_: str = attr("maxhullvert", default=None)
            name_: str = attr("name", default=None)
            normal_: str = attr("normal", default=None)
            refpos_: str = attr("refpos", default=None)
            refquat_: str = attr("refquat", default=None)
            scale_: str = attr("scale", default=None)
            smoothnormal_: str = attr("smoothnormal", default=None)
            texcoord_: str = attr("texcoord", default=None)
            vertex_: str = attr("vertex", default=None)

            class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
                instance_: str = attr("instance", default=None)
                plugin_: str = attr("plugin", default=None)

                class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                    key_: str = attr("key", default=None)
                    value_: str = attr("value", default=None)

                config_: List[Config] = []

            plugin_: List[Plugin] = []

        class Hfield(BaseXmlModel, tag="hfield", search_mode="unordered"):
            content_type_: str = attr("content_type", default=None)
            elevation_: str = attr("elevation", default=None)
            file_: str = attr("file", default=None)
            name_: str = attr("name", default=None)
            ncol_: str = attr("ncol", default=None)
            nrow_: str = attr("nrow", default=None)
            size_: str = attr("size", default=None)

        class Skin(BaseXmlModel, tag="skin", search_mode="unordered"):
            face_: str = attr("face", default=None)
            file_: str = attr("file", default=None)
            group_: str = attr("group", default=None)
            inflate_: str = attr("inflate", default=None)
            material_: str = attr("material", default=None)
            name_: str = attr("name", default=None)
            rgba_: str = attr("rgba", default=None)
            texcoord_: str = attr("texcoord", default=None)
            vertex_: str = attr("vertex", default=None)

            class Bone(BaseXmlModel, tag="bone", search_mode="unordered"):
                bindpos_: str = attr("bindpos", default=None)
                bindquat_: str = attr("bindquat", default=None)
                body_: str = attr("body", default=None)
                vertid_: str = attr("vertid", default=None)
                vertweight_: str = attr("vertweight", default=None)

            bone_: List[Bone] = []

        class Texture(BaseXmlModel, tag="texture", search_mode="unordered"):
            builtin_: str = attr("builtin", default=None)
            content_type_: str = attr("content_type", default=None)
            file_: str = attr("file", default=None)
            fileback_: str = attr("fileback", default=None)
            filedown_: str = attr("filedown", default=None)
            filefront_: str = attr("filefront", default=None)
            fileleft_: str = attr("fileleft", default=None)
            fileright_: str = attr("fileright", default=None)
            fileup_: str = attr("fileup", default=None)
            gridlayout_: str = attr("gridlayout", default=None)
            gridsize_: str = attr("gridsize", default=None)
            height_: str = attr("height", default=None)
            hflip_: str = attr("hflip", default=None)
            mark_: str = attr("mark", default=None)
            markrgb_: str = attr("markrgb", default=None)
            name_: str = attr("name", default=None)
            nchannel_: str = attr("nchannel", default=None)
            random_: str = attr("random", default=None)
            rgb1_: str = attr("rgb1", default=None)
            rgb2_: str = attr("rgb2", default=None)
            type_: str = attr("type", default=None)
            vflip_: str = attr("vflip", default=None)
            width_: str = attr("width", default=None)

        class Material(BaseXmlModel, tag="material", search_mode="unordered"):
            class_: str = attr("class", default=None)
            emission_: str = attr("emission", default=None)
            metallic_: str = attr("metallic", default=None)
            name_: str = attr("name", default=None)
            reflectance_: str = attr("reflectance", default=None)
            rgba_: str = attr("rgba", default=None)
            roughness_: str = attr("roughness", default=None)
            shininess_: str = attr("shininess", default=None)
            specular_: str = attr("specular", default=None)
            texrepeat_: str = attr("texrepeat", default=None)
            texture_: str = attr("texture", default=None)
            texuniform_: str = attr("texuniform", default=None)

            class Rgb(BaseXmlModel, tag="rgb", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Occlusion(BaseXmlModel, tag="occlusion", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Roughness(BaseXmlModel, tag="roughness", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Metallic(BaseXmlModel, tag="metallic", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Normal(BaseXmlModel, tag="normal", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Opacity(BaseXmlModel, tag="opacity", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Emissive(BaseXmlModel, tag="emissive", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Rgba(BaseXmlModel, tag="rgba", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            class Orm(BaseXmlModel, tag="orm", search_mode="unordered"):
                texture_: str = attr("texture", default=None)

            rgb_: Optional[Rgb] = None
            occlusion_: Optional[Occlusion] = None
            roughness_: Optional[Roughness] = None
            metallic_: Optional[Metallic] = None
            normal_: Optional[Normal] = None
            opacity_: Optional[Opacity] = None
            emissive_: Optional[Emissive] = None
            rgba_: Optional[Rgba] = None
            orm_: Optional[Orm] = None

        class Model(BaseXmlModel, tag="model", search_mode="unordered"):
            file_: str = attr("file", default=None)
            name_: str = attr("name", default=None)

        mesh_: List[Mesh] = []
        hfield_: List[Hfield] = []
        skin_: List[Skin] = []
        texture_: List[Texture] = []
        material_: List[Material] = []
        model_: List[Model] = []

    class Body(BaseXmlModel, tag="body", search_mode="unordered"):
        axisangle_: str = attr("axisangle", default=None)
        childclass_: str = attr("childclass", default=None)
        euler_: str = attr("euler", default=None)
        gravcomp_: str = attr("gravcomp", default=None)
        mocap_: str = attr("mocap", default=None)
        name_: str = attr("name", default=None)
        pos_: str = attr("pos", default=None)
        quat_: str = attr("quat", default=None)
        user_: str = attr("user", default=None)
        xyaxes_: str = attr("xyaxes", default=None)
        zaxis_: str = attr("zaxis", default=None)

        class Inertial(BaseXmlModel, tag="inertial", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            diaginertia_: str = attr("diaginertia", default=None)
            euler_: str = attr("euler", default=None)
            fullinertia_: str = attr("fullinertia", default=None)
            mass_: str = attr("mass", default=None)
            pos_: str = attr("pos", default=None)
            quat_: str = attr("quat", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

        class Joint(BaseXmlModel, tag="joint", search_mode="unordered"):
            actuatorfrclimited_: str = attr("actuatorfrclimited", default=None)
            actuatorfrcrange_: str = attr("actuatorfrcrange", default=None)
            actuatorgravcomp_: str = attr("actuatorgravcomp", default=None)
            armature_: str = attr("armature", default=None)
            axis_: str = attr("axis", default=None)
            class_: str = attr("class", default=None)
            damping_: str = attr("damping", default=None)
            frictionloss_: str = attr("frictionloss", default=None)
            group_: str = attr("group", default=None)
            limited_: str = attr("limited", default=None)
            margin_: str = attr("margin", default=None)
            name_: str = attr("name", default=None)
            pos_: str = attr("pos", default=None)
            range_: str = attr("range", default=None)
            ref_: str = attr("ref", default=None)
            solimpfriction_: str = attr("solimpfriction", default=None)
            solimplimit_: str = attr("solimplimit", default=None)
            solreffriction_: str = attr("solreffriction", default=None)
            solreflimit_: str = attr("solreflimit", default=None)
            springdamper_: str = attr("springdamper", default=None)
            springref_: str = attr("springref", default=None)
            stiffness_: str = attr("stiffness", default=None)
            type_: str = attr("type", default=None)
            user_: str = attr("user", default=None)

        class Freejoint(BaseXmlModel, tag="freejoint", search_mode="unordered"):
            group_: str = attr("group", default=None)
            name_: str = attr("name", default=None)

        class Geom(BaseXmlModel, tag="geom", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            class_: str = attr("class", default=None)
            conaffinity_: str = attr("conaffinity", default=None)
            condim_: str = attr("condim", default=None)
            contype_: str = attr("contype", default=None)
            density_: str = attr("density", default=None)
            euler_: str = attr("euler", default=None)
            fitscale_: str = attr("fitscale", default=None)
            fluidcoef_: str = attr("fluidcoef", default=None)
            fluidshape_: str = attr("fluidshape", default=None)
            friction_: str = attr("friction", default=None)
            fromto_: str = attr("fromto", default=None)
            gap_: str = attr("gap", default=None)
            group_: str = attr("group", default=None)
            hfield_: str = attr("hfield", default=None)
            margin_: str = attr("margin", default=None)
            mass_: str = attr("mass", default=None)
            material_: str = attr("material", default=None)
            mesh_: str = attr("mesh", default=None)
            name_: str = attr("name", default=None)
            pos_: str = attr("pos", default=None)
            priority_: str = attr("priority", default=None)
            quat_: str = attr("quat", default=None)
            rgba_: str = attr("rgba", default=None)
            shellinertia_: str = attr("shellinertia", default=None)
            size_: str = attr("size", default=None)
            solimp_: str = attr("solimp", default=None)
            solmix_: str = attr("solmix", default=None)
            solref_: str = attr("solref", default=None)
            type_: str = attr("type", default=None)
            user_: str = attr("user", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

            class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
                instance_: str = attr("instance", default=None)
                plugin_: str = attr("plugin", default=None)

                class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                    key_: str = attr("key", default=None)
                    value_: str = attr("value", default=None)

                config_: List[Config] = []

            plugin_: List[Plugin] = []

        class Attach(BaseXmlModel, tag="attach", search_mode="unordered"):
            body_: str = attr("body", default=None)
            model_: str = attr("model", default=None)
            prefix_: str = attr("prefix", default=None)

        class Site(BaseXmlModel, tag="site", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            class_: str = attr("class", default=None)
            euler_: str = attr("euler", default=None)
            fromto_: str = attr("fromto", default=None)
            group_: str = attr("group", default=None)
            material_: str = attr("material", default=None)
            name_: str = attr("name", default=None)
            pos_: str = attr("pos", default=None)
            quat_: str = attr("quat", default=None)
            rgba_: str = attr("rgba", default=None)
            size_: str = attr("size", default=None)
            type_: str = attr("type", default=None)
            user_: str = attr("user", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

        class Camera(BaseXmlModel, tag="camera", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            class_: str = attr("class", default=None)
            euler_: str = attr("euler", default=None)
            focal_: str = attr("focal", default=None)
            focalpixel_: str = attr("focalpixel", default=None)
            fovy_: str = attr("fovy", default=None)
            ipd_: str = attr("ipd", default=None)
            mode_: str = attr("mode", default=None)
            name_: str = attr("name", default=None)
            orthographic_: str = attr("orthographic", default=None)
            pos_: str = attr("pos", default=None)
            principal_: str = attr("principal", default=None)
            principalpixel_: str = attr("principalpixel", default=None)
            quat_: str = attr("quat", default=None)
            resolution_: str = attr("resolution", default=None)
            sensorsize_: str = attr("sensorsize", default=None)
            target_: str = attr("target", default=None)
            user_: str = attr("user", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

        class Light(BaseXmlModel, tag="light", search_mode="unordered"):
            active_: str = attr("active", default=None)
            ambient_: str = attr("ambient", default=None)
            attenuation_: str = attr("attenuation", default=None)
            bulbradius_: str = attr("bulbradius", default=None)
            castshadow_: str = attr("castshadow", default=None)
            class_: str = attr("class", default=None)
            cutoff_: str = attr("cutoff", default=None)
            diffuse_: str = attr("diffuse", default=None)
            dir_: str = attr("dir", default=None)
            directional_: str = attr("directional", default=None)
            exponent_: str = attr("exponent", default=None)
            mode_: str = attr("mode", default=None)
            name_: str = attr("name", default=None)
            pos_: str = attr("pos", default=None)
            specular_: str = attr("specular", default=None)
            target_: str = attr("target", default=None)

        class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
            instance_: str = attr("instance", default=None)
            plugin_: str = attr("plugin", default=None)

            class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                key_: str = attr("key", default=None)
                value_: str = attr("value", default=None)

            config_: List[Config] = []

        class Composite(BaseXmlModel, tag="composite", search_mode="unordered"):
            count_: str = attr("count", default=None)
            curve_: str = attr("curve", default=None)
            face_: str = attr("face", default=None)
            flatinertia_: str = attr("flatinertia", default=None)
            initial_: str = attr("initial", default=None)
            offset_: str = attr("offset", default=None)
            prefix_: str = attr("prefix", default=None)
            size_: str = attr("size", default=None)
            solimpsmooth_: str = attr("solimpsmooth", default=None)
            solrefsmooth_: str = attr("solrefsmooth", default=None)
            spacing_: str = attr("spacing", default=None)
            type_: str = attr("type", default=None)
            vertex_: str = attr("vertex", default=None)

            class Joint(BaseXmlModel, tag="joint", search_mode="unordered"):
                armature_: str = attr("armature", default=None)
                axis_: str = attr("axis", default=None)
                damping_: str = attr("damping", default=None)
                frictionloss_: str = attr("frictionloss", default=None)
                group_: str = attr("group", default=None)
                kind_: str = attr("kind", default=None)
                limited_: str = attr("limited", default=None)
                margin_: str = attr("margin", default=None)
                range_: str = attr("range", default=None)
                solimpfix_: str = attr("solimpfix", default=None)
                solimpfriction_: str = attr("solimpfriction", default=None)
                solimplimit_: str = attr("solimplimit", default=None)
                solreffix_: str = attr("solreffix", default=None)
                solreffriction_: str = attr("solreffriction", default=None)
                solreflimit_: str = attr("solreflimit", default=None)
                stiffness_: str = attr("stiffness", default=None)
                type_: str = attr("type", default=None)

            class Tendon(BaseXmlModel, tag="tendon", search_mode="unordered"):
                damping_: str = attr("damping", default=None)
                frictionloss_: str = attr("frictionloss", default=None)
                group_: str = attr("group", default=None)
                kind_: str = attr("kind", default=None)
                limited_: str = attr("limited", default=None)
                margin_: str = attr("margin", default=None)
                material_: str = attr("material", default=None)
                range_: str = attr("range", default=None)
                rgba_: str = attr("rgba", default=None)
                solimpfix_: str = attr("solimpfix", default=None)
                solimpfriction_: str = attr("solimpfriction", default=None)
                solimplimit_: str = attr("solimplimit", default=None)
                solreffix_: str = attr("solreffix", default=None)
                solreffriction_: str = attr("solreffriction", default=None)
                solreflimit_: str = attr("solreflimit", default=None)
                stiffness_: str = attr("stiffness", default=None)
                width_: str = attr("width", default=None)

            class Skin(BaseXmlModel, tag="skin", search_mode="unordered"):
                group_: str = attr("group", default=None)
                inflate_: str = attr("inflate", default=None)
                material_: str = attr("material", default=None)
                rgba_: str = attr("rgba", default=None)
                subgrid_: str = attr("subgrid", default=None)
                texcoord_: str = attr("texcoord", default=None)

            class Geom(BaseXmlModel, tag="geom", search_mode="unordered"):
                conaffinity_: str = attr("conaffinity", default=None)
                condim_: str = attr("condim", default=None)
                contype_: str = attr("contype", default=None)
                density_: str = attr("density", default=None)
                friction_: str = attr("friction", default=None)
                gap_: str = attr("gap", default=None)
                group_: str = attr("group", default=None)
                margin_: str = attr("margin", default=None)
                mass_: str = attr("mass", default=None)
                material_: str = attr("material", default=None)
                priority_: str = attr("priority", default=None)
                rgba_: str = attr("rgba", default=None)
                size_: str = attr("size", default=None)
                solimp_: str = attr("solimp", default=None)
                solmix_: str = attr("solmix", default=None)
                solref_: str = attr("solref", default=None)
                type_: str = attr("type", default=None)

            class Site(BaseXmlModel, tag="site", search_mode="unordered"):
                group_: str = attr("group", default=None)
                material_: str = attr("material", default=None)
                rgba_: str = attr("rgba", default=None)
                size_: str = attr("size", default=None)

            class Pin(BaseXmlModel, tag="pin", search_mode="unordered"):
                coord_: str = attr("coord", default=None)

            class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
                instance_: str = attr("instance", default=None)
                plugin_: str = attr("plugin", default=None)

                class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                    key_: str = attr("key", default=None)
                    value_: str = attr("value", default=None)

                config_: List[Config] = []

            joint_: List[Joint] = []
            tendon_: List[Tendon] = []
            skin_: Optional[Skin] = None
            geom_: Optional[Geom] = None
            site_: Optional[Site] = None
            pin_: List[Pin] = []
            plugin_: List[Plugin] = []

        class Flexcomp(BaseXmlModel, tag="flexcomp", search_mode="unordered"):
            axisangle_: str = attr("axisangle", default=None)
            count_: str = attr("count", default=None)
            dim_: str = attr("dim", default=None)
            element_: str = attr("element", default=None)
            euler_: str = attr("euler", default=None)
            file_: str = attr("file", default=None)
            flatskin_: str = attr("flatskin", default=None)
            group_: str = attr("group", default=None)
            inertiabox_: str = attr("inertiabox", default=None)
            mass_: str = attr("mass", default=None)
            material_: str = attr("material", default=None)
            name_: str = attr("name", default=None)
            point_: str = attr("point", default=None)
            pos_: str = attr("pos", default=None)
            quat_: str = attr("quat", default=None)
            radius_: str = attr("radius", default=None)
            rgba_: str = attr("rgba", default=None)
            rigid_: str = attr("rigid", default=None)
            scale_: str = attr("scale", default=None)
            spacing_: str = attr("spacing", default=None)
            texcoord_: str = attr("texcoord", default=None)
            type_: str = attr("type", default=None)
            xyaxes_: str = attr("xyaxes", default=None)
            zaxis_: str = attr("zaxis", default=None)

            class Edge(BaseXmlModel, tag="edge", search_mode="unordered"):
                damping_: str = attr("damping", default=None)
                equality_: str = attr("equality", default=None)
                solimp_: str = attr("solimp", default=None)
                solref_: str = attr("solref", default=None)
                stiffness_: str = attr("stiffness", default=None)

            class Contact(BaseXmlModel, tag="contact", search_mode="unordered"):
                activelayers_: str = attr("activelayers", default=None)
                conaffinity_: str = attr("conaffinity", default=None)
                condim_: str = attr("condim", default=None)
                contype_: str = attr("contype", default=None)
                friction_: str = attr("friction", default=None)
                gap_: str = attr("gap", default=None)
                internal_: str = attr("internal", default=None)
                margin_: str = attr("margin", default=None)
                priority_: str = attr("priority", default=None)
                selfcollide_: str = attr("selfcollide", default=None)
                solimp_: str = attr("solimp", default=None)
                solmix_: str = attr("solmix", default=None)
                solref_: str = attr("solref", default=None)

            class Pin(BaseXmlModel, tag="pin", search_mode="unordered"):
                grid_: str = attr("grid", default=None)
                gridrange_: str = attr("gridrange", default=None)
                id_: str = attr("id", default=None)
                range_: str = attr("range", default=None)

            class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
                instance_: str = attr("instance", default=None)
                plugin_: str = attr("plugin", default=None)

                class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                    key_: str = attr("key", default=None)
                    value_: str = attr("value", default=None)

                config_: List[Config] = []

            edge_: Optional[Edge] = None
            contact_: Optional[Contact] = None
            pin_: List[Pin] = []
            plugin_: List[Plugin] = []

        body_: List["Body"] = []

        inertial_: Optional[Inertial] = None
        joint_: List[Joint] = []
        freejoint_: List[Freejoint] = []
        geom_: List[Geom] = []
        attach_: List[Attach] = []
        site_: List[Site] = []
        camera_: List[Camera] = []
        light_: List[Light] = []
        plugin_: List[Plugin] = []
        composite_: List[Composite] = []
        flexcomp_: List[Flexcomp] = []

    class Deformable(BaseXmlModel, tag="deformable", search_mode="unordered"):

        class Flex(BaseXmlModel, tag="flex", search_mode="unordered"):
            body_: str = attr("body", default=None)
            dim_: str = attr("dim", default=None)
            element_: str = attr("element", default=None)
            flatskin_: str = attr("flatskin", default=None)
            group_: str = attr("group", default=None)
            material_: str = attr("material", default=None)
            name_: str = attr("name", default=None)
            radius_: str = attr("radius", default=None)
            rgba_: str = attr("rgba", default=None)
            texcoord_: str = attr("texcoord", default=None)
            vertex_: str = attr("vertex", default=None)

            class Contact(BaseXmlModel, tag="contact", search_mode="unordered"):
                activelayers_: str = attr("activelayers", default=None)
                conaffinity_: str = attr("conaffinity", default=None)
                condim_: str = attr("condim", default=None)
                contype_: str = attr("contype", default=None)
                friction_: str = attr("friction", default=None)
                gap_: str = attr("gap", default=None)
                internal_: str = attr("internal", default=None)
                margin_: str = attr("margin", default=None)
                priority_: str = attr("priority", default=None)
                selfcollide_: str = attr("selfcollide", default=None)
                solimp_: str = attr("solimp", default=None)
                solmix_: str = attr("solmix", default=None)
                solref_: str = attr("solref", default=None)

            class Edge(BaseXmlModel, tag="edge", search_mode="unordered"):
                damping_: str = attr("damping", default=None)
                stiffness_: str = attr("stiffness", default=None)

            contact_: Optional[Contact] = None
            edge_: Optional[Edge] = None

        class Skin(BaseXmlModel, tag="skin", search_mode="unordered"):
            face_: str = attr("face", default=None)
            file_: str = attr("file", default=None)
            group_: str = attr("group", default=None)
            inflate_: str = attr("inflate", default=None)
            material_: str = attr("material", default=None)
            name_: str = attr("name", default=None)
            rgba_: str = attr("rgba", default=None)
            texcoord_: str = attr("texcoord", default=None)
            vertex_: str = attr("vertex", default=None)

            class Bone(BaseXmlModel, tag="bone", search_mode="unordered"):
                bindpos_: str = attr("bindpos", default=None)
                bindquat_: str = attr("bindquat", default=None)
                body_: str = attr("body", default=None)
                vertid_: str = attr("vertid", default=None)
                vertweight_: str = attr("vertweight", default=None)

            bone_: List[Bone] = []

        flex_: List[Flex] = []
        skin_: List[Skin] = []

    class Contact(BaseXmlModel, tag="contact", search_mode="unordered"):

        class Pair(BaseXmlModel, tag="pair", search_mode="unordered"):
            class_: str = attr("class", default=None)
            condim_: str = attr("condim", default=None)
            friction_: str = attr("friction", default=None)
            gap_: str = attr("gap", default=None)
            geom1_: str = attr("geom1", default=None)
            geom2_: str = attr("geom2", default=None)
            margin_: str = attr("margin", default=None)
            name_: str = attr("name", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)
            solreffriction_: str = attr("solreffriction", default=None)

        class Exclude(BaseXmlModel, tag="exclude", search_mode="unordered"):
            body1_: str = attr("body1", default=None)
            body2_: str = attr("body2", default=None)
            name_: str = attr("name", default=None)

        pair_: List[Pair] = []
        exclude_: List[Exclude] = []

    class Equality(BaseXmlModel, tag="equality", search_mode="unordered"):

        class Connect(BaseXmlModel, tag="connect", search_mode="unordered"):
            active_: str = attr("active", default=None)
            anchor_: str = attr("anchor", default=None)
            body1_: str = attr("body1", default=None)
            body2_: str = attr("body2", default=None)
            class_: str = attr("class", default=None)
            name_: str = attr("name", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)

        class Weld(BaseXmlModel, tag="weld", search_mode="unordered"):
            active_: str = attr("active", default=None)
            anchor_: str = attr("anchor", default=None)
            body1_: str = attr("body1", default=None)
            body2_: str = attr("body2", default=None)
            class_: str = attr("class", default=None)
            name_: str = attr("name", default=None)
            relpose_: str = attr("relpose", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)
            torquescale_: str = attr("torquescale", default=None)

        class Joint(BaseXmlModel, tag="joint", search_mode="unordered"):
            active_: str = attr("active", default=None)
            class_: str = attr("class", default=None)
            joint1_: str = attr("joint1", default=None)
            joint2_: str = attr("joint2", default=None)
            name_: str = attr("name", default=None)
            polycoef_: str = attr("polycoef", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)

        class Tendon(BaseXmlModel, tag="tendon", search_mode="unordered"):
            active_: str = attr("active", default=None)
            class_: str = attr("class", default=None)
            name_: str = attr("name", default=None)
            polycoef_: str = attr("polycoef", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)
            tendon1_: str = attr("tendon1", default=None)
            tendon2_: str = attr("tendon2", default=None)

        class Flex(BaseXmlModel, tag="flex", search_mode="unordered"):
            active_: str = attr("active", default=None)
            class_: str = attr("class", default=None)
            flex_: str = attr("flex", default=None)
            name_: str = attr("name", default=None)
            solimp_: str = attr("solimp", default=None)
            solref_: str = attr("solref", default=None)

        connect_: List[Connect] = []
        weld_: List[Weld] = []
        joint_: List[Joint] = []
        tendon_: List[Tendon] = []
        flex_: List[Flex] = []

    class Tendon(BaseXmlModel, tag="tendon", search_mode="unordered"):

        class Spatial(BaseXmlModel, tag="spatial", search_mode="unordered"):
            class_: str = attr("class", default=None)
            damping_: str = attr("damping", default=None)
            frictionloss_: str = attr("frictionloss", default=None)
            group_: str = attr("group", default=None)
            limited_: str = attr("limited", default=None)
            margin_: str = attr("margin", default=None)
            material_: str = attr("material", default=None)
            name_: str = attr("name", default=None)
            range_: str = attr("range", default=None)
            rgba_: str = attr("rgba", default=None)
            solimpfriction_: str = attr("solimpfriction", default=None)
            solimplimit_: str = attr("solimplimit", default=None)
            solreffriction_: str = attr("solreffriction", default=None)
            solreflimit_: str = attr("solreflimit", default=None)
            springlength_: str = attr("springlength", default=None)
            stiffness_: str = attr("stiffness", default=None)
            user_: str = attr("user", default=None)
            width_: str = attr("width", default=None)

            class Site(BaseXmlModel, tag="site", search_mode="unordered"):
                site_: str = attr("site", default=None)

            class Geom(BaseXmlModel, tag="geom", search_mode="unordered"):
                geom_: str = attr("geom", default=None)
                sidesite_: str = attr("sidesite", default=None)

            class Pulley(BaseXmlModel, tag="pulley", search_mode="unordered"):
                divisor_: str = attr("divisor", default=None)

            site_: List[Site] = []
            geom_: List[Geom] = []
            pulley_: List[Pulley] = []

        class Fixed(BaseXmlModel, tag="fixed", search_mode="unordered"):
            class_: str = attr("class", default=None)
            damping_: str = attr("damping", default=None)
            frictionloss_: str = attr("frictionloss", default=None)
            group_: str = attr("group", default=None)
            limited_: str = attr("limited", default=None)
            margin_: str = attr("margin", default=None)
            name_: str = attr("name", default=None)
            range_: str = attr("range", default=None)
            solimpfriction_: str = attr("solimpfriction", default=None)
            solimplimit_: str = attr("solimplimit", default=None)
            solreffriction_: str = attr("solreffriction", default=None)
            solreflimit_: str = attr("solreflimit", default=None)
            springlength_: str = attr("springlength", default=None)
            stiffness_: str = attr("stiffness", default=None)
            user_: str = attr("user", default=None)

            class Joint(BaseXmlModel, tag="joint", search_mode="unordered"):
                coef_: str = attr("coef", default=None)
                joint_: str = attr("joint", default=None)

            joint_: List[Joint] = []

        spatial_: List[Spatial] = []
        fixed_: List[Fixed] = []

    class Actuator(BaseXmlModel, tag="actuator", search_mode="unordered"):

        class General(BaseXmlModel, tag="general", search_mode="unordered"):
            actdim_: str = attr("actdim", default=None)
            actearly_: str = attr("actearly", default=None)
            actlimited_: str = attr("actlimited", default=None)
            actrange_: str = attr("actrange", default=None)
            biasprm_: str = attr("biasprm", default=None)
            biastype_: str = attr("biastype", default=None)
            body_: str = attr("body", default=None)
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            dynprm_: str = attr("dynprm", default=None)
            dyntype_: str = attr("dyntype", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gainprm_: str = attr("gainprm", default=None)
            gaintype_: str = attr("gaintype", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            refsite_: str = attr("refsite", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Motor(BaseXmlModel, tag="motor", search_mode="unordered"):
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            refsite_: str = attr("refsite", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Position(BaseXmlModel, tag="position", search_mode="unordered"):
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            dampratio_: str = attr("dampratio", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            inheritrange_: str = attr("inheritrange", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            kp_: str = attr("kp", default=None)
            kv_: str = attr("kv", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            refsite_: str = attr("refsite", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            timeconst_: str = attr("timeconst", default=None)
            user_: str = attr("user", default=None)

        class Velocity(BaseXmlModel, tag="velocity", search_mode="unordered"):
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            kv_: str = attr("kv", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            refsite_: str = attr("refsite", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Intvelocity(BaseXmlModel, tag="intvelocity", search_mode="unordered"):
            actrange_: str = attr("actrange", default=None)
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            dampratio_: str = attr("dampratio", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            inheritrange_: str = attr("inheritrange", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            kp_: str = attr("kp", default=None)
            kv_: str = attr("kv", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            refsite_: str = attr("refsite", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Damper(BaseXmlModel, tag="damper", search_mode="unordered"):
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            kv_: str = attr("kv", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            refsite_: str = attr("refsite", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Cylinder(BaseXmlModel, tag="cylinder", search_mode="unordered"):
            area_: str = attr("area", default=None)
            bias_: str = attr("bias", default=None)
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            diameter_: str = attr("diameter", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            refsite_: str = attr("refsite", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            timeconst_: str = attr("timeconst", default=None)
            user_: str = attr("user", default=None)

        class Muscle(BaseXmlModel, tag="muscle", search_mode="unordered"):
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            force_: str = attr("force", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            fpmax_: str = attr("fpmax", default=None)
            fvmax_: str = attr("fvmax", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            lmax_: str = attr("lmax", default=None)
            lmin_: str = attr("lmin", default=None)
            name_: str = attr("name", default=None)
            range_: str = attr("range", default=None)
            scale_: str = attr("scale", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tausmooth_: str = attr("tausmooth", default=None)
            tendon_: str = attr("tendon", default=None)
            timeconst_: str = attr("timeconst", default=None)
            user_: str = attr("user", default=None)
            vmax_: str = attr("vmax", default=None)

        class Adhesion(BaseXmlModel, tag="adhesion", search_mode="unordered"):
            body_: str = attr("body", default=None)
            class_: str = attr("class", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gain_: str = attr("gain", default=None)
            group_: str = attr("group", default=None)
            name_: str = attr("name", default=None)
            user_: str = attr("user", default=None)

        class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
            actdim_: str = attr("actdim", default=None)
            actearly_: str = attr("actearly", default=None)
            actlimited_: str = attr("actlimited", default=None)
            actrange_: str = attr("actrange", default=None)
            class_: str = attr("class", default=None)
            cranklength_: str = attr("cranklength", default=None)
            cranksite_: str = attr("cranksite", default=None)
            ctrllimited_: str = attr("ctrllimited", default=None)
            ctrlrange_: str = attr("ctrlrange", default=None)
            dynprm_: str = attr("dynprm", default=None)
            dyntype_: str = attr("dyntype", default=None)
            forcelimited_: str = attr("forcelimited", default=None)
            forcerange_: str = attr("forcerange", default=None)
            gear_: str = attr("gear", default=None)
            group_: str = attr("group", default=None)
            instance_: str = attr("instance", default=None)
            joint_: str = attr("joint", default=None)
            jointinparent_: str = attr("jointinparent", default=None)
            lengthrange_: str = attr("lengthrange", default=None)
            name_: str = attr("name", default=None)
            plugin_: str = attr("plugin", default=None)
            site_: str = attr("site", default=None)
            slidersite_: str = attr("slidersite", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

            class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                key_: str = attr("key", default=None)
                value_: str = attr("value", default=None)

            config_: List[Config] = []

        general_: List[General] = []
        motor_: List[Motor] = []
        position_: List[Position] = []
        velocity_: List[Velocity] = []
        intvelocity_: List[Intvelocity] = []
        damper_: List[Damper] = []
        cylinder_: List[Cylinder] = []
        muscle_: List[Muscle] = []
        adhesion_: List[Adhesion] = []
        plugin_: List[Plugin] = []

    class Sensor(BaseXmlModel, tag="sensor", search_mode="unordered"):

        class Touch(BaseXmlModel, tag="touch", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Accelerometer(BaseXmlModel, tag="accelerometer", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Velocimeter(BaseXmlModel, tag="velocimeter", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Gyro(BaseXmlModel, tag="gyro", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Force(BaseXmlModel, tag="force", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Torque(BaseXmlModel, tag="torque", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Magnetometer(BaseXmlModel, tag="magnetometer", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Camprojection(BaseXmlModel, tag="camprojection", search_mode="unordered"):
            camera_: str = attr("camera", default=None)
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Rangefinder(BaseXmlModel, tag="rangefinder", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            site_: str = attr("site", default=None)
            user_: str = attr("user", default=None)

        class Jointpos(BaseXmlModel, tag="jointpos", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Jointvel(BaseXmlModel, tag="jointvel", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Tendonpos(BaseXmlModel, tag="tendonpos", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Tendonvel(BaseXmlModel, tag="tendonvel", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Actuatorpos(BaseXmlModel, tag="actuatorpos", search_mode="unordered"):
            actuator_: str = attr("actuator", default=None)
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Actuatorvel(BaseXmlModel, tag="actuatorvel", search_mode="unordered"):
            actuator_: str = attr("actuator", default=None)
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Actuatorfrc(BaseXmlModel, tag="actuatorfrc", search_mode="unordered"):
            actuator_: str = attr("actuator", default=None)
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Jointactuatorfrc(BaseXmlModel, tag="jointactuatorfrc", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Ballquat(BaseXmlModel, tag="ballquat", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Ballangvel(BaseXmlModel, tag="ballangvel", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Jointlimitpos(BaseXmlModel, tag="jointlimitpos", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Jointlimitvel(BaseXmlModel, tag="jointlimitvel", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Jointlimitfrc(BaseXmlModel, tag="jointlimitfrc", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            joint_: str = attr("joint", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Tendonlimitpos(BaseXmlModel, tag="tendonlimitpos", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Tendonlimitvel(BaseXmlModel, tag="tendonlimitvel", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Tendonlimitfrc(BaseXmlModel, tag="tendonlimitfrc", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            tendon_: str = attr("tendon", default=None)
            user_: str = attr("user", default=None)

        class Framepos(BaseXmlModel, tag="framepos", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

        class Framequat(BaseXmlModel, tag="framequat", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

        class Framexaxis(BaseXmlModel, tag="framexaxis", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

        class Frameyaxis(BaseXmlModel, tag="frameyaxis", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

        class Framezaxis(BaseXmlModel, tag="framezaxis", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

        class Framelinvel(BaseXmlModel, tag="framelinvel", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

        class Frameangvel(BaseXmlModel, tag="frameangvel", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

        class Framelinacc(BaseXmlModel, tag="framelinacc", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            user_: str = attr("user", default=None)

        class Frameangacc(BaseXmlModel, tag="frameangacc", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            user_: str = attr("user", default=None)

        class Subtreecom(BaseXmlModel, tag="subtreecom", search_mode="unordered"):
            body_: str = attr("body", default=None)
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Subtreelinvel(BaseXmlModel, tag="subtreelinvel", search_mode="unordered"):
            body_: str = attr("body", default=None)
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Subtreeangmom(BaseXmlModel, tag="subtreeangmom", search_mode="unordered"):
            body_: str = attr("body", default=None)
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Distance(BaseXmlModel, tag="distance", search_mode="unordered"):
            body1_: str = attr("body1", default=None)
            body2_: str = attr("body2", default=None)
            cutoff_: str = attr("cutoff", default=None)
            geom1_: str = attr("geom1", default=None)
            geom2_: str = attr("geom2", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Normal(BaseXmlModel, tag="normal", search_mode="unordered"):
            body1_: str = attr("body1", default=None)
            body2_: str = attr("body2", default=None)
            cutoff_: str = attr("cutoff", default=None)
            geom1_: str = attr("geom1", default=None)
            geom2_: str = attr("geom2", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Fromto(BaseXmlModel, tag="fromto", search_mode="unordered"):
            body1_: str = attr("body1", default=None)
            body2_: str = attr("body2", default=None)
            cutoff_: str = attr("cutoff", default=None)
            geom1_: str = attr("geom1", default=None)
            geom2_: str = attr("geom2", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class Clock(BaseXmlModel, tag="clock", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            name_: str = attr("name", default=None)
            noise_: str = attr("noise", default=None)
            user_: str = attr("user", default=None)

        class User(BaseXmlModel, tag="user", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            datatype_: str = attr("datatype", default=None)
            dim_: str = attr("dim", default=None)
            name_: str = attr("name", default=None)
            needstage_: str = attr("needstage", default=None)
            noise_: str = attr("noise", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            user_: str = attr("user", default=None)

        class Plugin(BaseXmlModel, tag="plugin", search_mode="unordered"):
            cutoff_: str = attr("cutoff", default=None)
            instance_: str = attr("instance", default=None)
            name_: str = attr("name", default=None)
            objname_: str = attr("objname", default=None)
            objtype_: str = attr("objtype", default=None)
            plugin_: str = attr("plugin", default=None)
            refname_: str = attr("refname", default=None)
            reftype_: str = attr("reftype", default=None)
            user_: str = attr("user", default=None)

            class Config(BaseXmlModel, tag="config", search_mode="unordered"):
                key_: str = attr("key", default=None)
                value_: str = attr("value", default=None)

            config_: List[Config] = []

        touch_: List[Touch] = []
        accelerometer_: List[Accelerometer] = []
        velocimeter_: List[Velocimeter] = []
        gyro_: List[Gyro] = []
        force_: List[Force] = []
        torque_: List[Torque] = []
        magnetometer_: List[Magnetometer] = []
        camprojection_: List[Camprojection] = []
        rangefinder_: List[Rangefinder] = []
        jointpos_: List[Jointpos] = []
        jointvel_: List[Jointvel] = []
        tendonpos_: List[Tendonpos] = []
        tendonvel_: List[Tendonvel] = []
        actuatorpos_: List[Actuatorpos] = []
        actuatorvel_: List[Actuatorvel] = []
        actuatorfrc_: List[Actuatorfrc] = []
        jointactuatorfrc_: List[Jointactuatorfrc] = []
        ballquat_: List[Ballquat] = []
        ballangvel_: List[Ballangvel] = []
        jointlimitpos_: List[Jointlimitpos] = []
        jointlimitvel_: List[Jointlimitvel] = []
        jointlimitfrc_: List[Jointlimitfrc] = []
        tendonlimitpos_: List[Tendonlimitpos] = []
        tendonlimitvel_: List[Tendonlimitvel] = []
        tendonlimitfrc_: List[Tendonlimitfrc] = []
        framepos_: List[Framepos] = []
        framequat_: List[Framequat] = []
        framexaxis_: List[Framexaxis] = []
        frameyaxis_: List[Frameyaxis] = []
        framezaxis_: List[Framezaxis] = []
        framelinvel_: List[Framelinvel] = []
        frameangvel_: List[Frameangvel] = []
        framelinacc_: List[Framelinacc] = []
        frameangacc_: List[Frameangacc] = []
        subtreecom_: List[Subtreecom] = []
        subtreelinvel_: List[Subtreelinvel] = []
        subtreeangmom_: List[Subtreeangmom] = []
        distance_: List[Distance] = []
        normal_: List[Normal] = []
        fromto_: List[Fromto] = []
        clock_: List[Clock] = []
        user_: List[User] = []
        plugin_: List[Plugin] = []

    class Keyframe(BaseXmlModel, tag="keyframe", search_mode="unordered"):

        class Key(BaseXmlModel, tag="key", search_mode="unordered"):
            act_: str = attr("act", default=None)
            ctrl_: str = attr("ctrl", default=None)
            mpos_: str = attr("mpos", default=None)
            mquat_: str = attr("mquat", default=None)
            name_: str = attr("name", default=None)
            qpos_: str = attr("qpos", default=None)
            qvel_: str = attr("qvel", default=None)
            time_: str = attr("time", default=None)

        key_: List[Key] = []

    compiler_: List[Compiler] = []
    option_: List[Option] = []
    size_: List[Size] = []
    visual_: List[Visual] = []
    statistic_: List[Statistic] = []
    default_: Optional[Default] = None
    extension_: List[Extension] = []
    custom_: List[Custom] = []
    asset_: List[Asset] = []

    class Worldbody(Body, tag="worldbody", search_mode="unordered"):
        pass

    worldbody_: Optional[Worldbody] = None
    deformable_: List[Deformable] = []
    contact_: List[Contact] = []
    equality_: List[Equality] = []
    tendon_: List[Tendon] = []
    actuator_: List[Actuator] = []
    sensor_: List[Sensor] = []
    keyframe_: List[Keyframe] = []


from pydantic_mujoco.extensions import *
