use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_xpbd_3d::prelude::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            WorldInspectorPlugin::new(),
        ))
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .add_systems(Startup, setup)
        .add_systems(Update, car_jump)
        .add_systems(FixedUpdate, (cam_follow_car, do_suspension_stuff))
        .run();
}

#[derive(Component)]
struct Car {
    suspension_rest_dist: f32,
    spring_strenght: f32,
    spring_damper: f32,
    wheel_radius: f32,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            suspension_rest_dist: 0.5,
            spring_strenght: 50.0,
            spring_damper: 10.0,
            wheel_radius: 0.33,
        }
    }
}

#[derive(Component)]
struct Wheel;

#[derive(Component)]
struct FrWheel;

#[derive(Component)]
struct FlWheel;

#[derive(Component)]
struct BrWheel;

#[derive(Component)]
struct BlWheel;

#[derive(Component)]
struct Ground;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::splat(5.0)).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh().size(50.0, 50.0)),
            material: materials.add(Color::GREEN),
            ..default()
        })
        .insert(RigidBody::Static)
        .insert(AsyncCollider(ComputedCollider::TriMesh))
        .insert(Friction::new(1.0))
        .insert(Ground);

    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Cuboid::from_size(Vec3::new(1.5, 0.4, 3.0))),
            material: materials.add(Color::WHITE),
            transform: Transform::from_translation(Vec3::Y * 1.5),
            ..default()
        })
        .insert(RigidBody::Dynamic)
        .insert(AsyncCollider(ComputedCollider::TriMesh))
        .insert(Friction::new(1.0))
        .insert(ExternalForce::default().with_persistence(false))
        // .insert(LockedAxes::ROTATION_LOCKED)
        // .insert(AngularDamping(5.0))
        .insert(SleepingDisabled)
        .insert(Car::default())
        .insert(Name::new("Car"))
        .with_children(|parent| {
            let max_toi = 1.0;
            let ray_cast_x = 0.7;
            let ray_cast_z = 0.95;

            let parent_entity = parent.parent_entity();

            parent
                .spawn(SpatialBundle {
                    transform: Transform::from_xyz(-ray_cast_x, 0.0, ray_cast_z),
                    ..default()
                })
                .insert(
                    RayCaster::new(Vec3::ZERO, Direction3d::NEG_Y)
                        .with_max_time_of_impact(max_toi)
                        .with_max_hits(1)
                        .with_query_filter(SpatialQueryFilter::from_excluded_entities([
                            parent_entity,
                        ])),
                )
                .insert(Wheel)
                .insert(FlWheel)
                .insert(Name::new("FL_Wheel"));

            parent
                .spawn(SpatialBundle {
                    transform: Transform::from_xyz(ray_cast_x, 0.0, ray_cast_z),
                    ..default()
                })
                .insert(
                    RayCaster::new(Vec3::ZERO, Direction3d::NEG_Y)
                        .with_max_time_of_impact(max_toi)
                        .with_max_hits(1)
                        .with_query_filter(SpatialQueryFilter::from_excluded_entities([
                            parent_entity,
                        ])),
                )
                .insert(Wheel)
                .insert(FrWheel)
                .insert(Name::new("FR_Wheel"));

            parent
                .spawn(SpatialBundle {
                    transform: Transform::from_xyz(-ray_cast_x, 0.0, -ray_cast_z),
                    ..default()
                })
                .insert(
                    RayCaster::new(Vec3::ZERO, Direction3d::NEG_Y)
                        .with_max_time_of_impact(max_toi)
                        .with_max_hits(1)
                        .with_query_filter(SpatialQueryFilter::from_excluded_entities([
                            parent_entity,
                        ])),
                )
                .insert(Wheel)
                .insert(BlWheel)
                .insert(Name::new("BL_Wheel"));

            parent
                .spawn(SpatialBundle {
                    transform: Transform::from_xyz(ray_cast_x, 0.0, -ray_cast_z),
                    ..default()
                })
                .insert(
                    RayCaster::new(Vec3::ZERO, Direction3d::NEG_Y)
                        .with_max_time_of_impact(max_toi)
                        .with_max_hits(1)
                        .with_query_filter(SpatialQueryFilter::from_excluded_entities([
                            parent_entity,
                        ])),
                )
                .insert(Wheel)
                .insert(BrWheel)
                .insert(Name::new("BR_Wheel"));
        });
}

fn do_suspension_stuff(
    mut car_q: Query<(&Car, &GlobalTransform, &mut ExternalForce)>,
    wheel_q: Query<(&RayCaster, &RayHits, &GlobalTransform), With<Wheel>>,
    mut previous_spring_lenght: Local<f32>,
    time: Res<Time>,
    mut gizmos: Gizmos,
) {
    if let Ok((car, car_global_transform, mut external_force)) = car_q.get_single_mut() {
        for (ray, hits, global_transform) in wheel_q.iter() {
            println!("len {}", hits.len());

            if let Some(hit) = hits.iter().next() {
                // the direction the force will be applied
                let susp_dir = global_transform.down();

                let ray_origin = ray.global_origin();
                let raycast_dest = ray_origin + ray.global_direction() * hit.time_of_impact;
                let distance = raycast_dest.distance(ray_origin);

                let contact = raycast_dest - car_global_transform.translation();

                let spring_lenght =
                    (distance - car.wheel_radius).clamp(0.0, car.suspension_rest_dist);

                let spring_force = car.spring_strenght * (car.suspension_rest_dist - spring_lenght);

                let spring_velocity =
                    (*previous_spring_lenght - spring_lenght) / time.delta_seconds();

                let damper_force = car.spring_damper * spring_velocity;

                let suspension_force = ray.direction.trunc() * (spring_force + damper_force);

                *previous_spring_lenght = spring_lenght;

                let point = Vec3::new(
                    raycast_dest.x,
                    raycast_dest.y + car.wheel_radius,
                    raycast_dest.z,
                );

                // println!("suspension force: {}", suspension_force);

                gizmos.arrow(point, susp_dir * suspension_force * 100.0, Color::BLUE);

                // Apply force
                external_force.apply_force_at_point(susp_dir * suspension_force, point, Vec3::ZERO);
                // external_force.apply_force_at_point(
                //     susp_dir * suspension_force,
                //     global_transform.translation(),
                //     Vec3::ZERO,
                // );
            }
        }
    }
}

fn car_jump(keys: Res<ButtonInput<KeyCode>>, mut car_q: Query<&mut LinearVelocity, With<Car>>) {
    if let Ok(mut vel) = car_q.get_single_mut() {
        if keys.pressed(KeyCode::Space) {
            vel.y = 2.5;
        }
    }
}

fn cam_follow_car(
    mut camera_q: Query<&mut Transform, (With<Camera3d>, Without<Car>)>,
    car_q: Query<&Transform, With<Car>>,
) {
    if let Ok(car) = car_q.get_single() {
        if let Ok(mut camera) = camera_q.get_single_mut() {
            camera.look_at(car.translation, Vec3::Y);
        }
    }
}
