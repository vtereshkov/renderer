package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"time"
)

type Vec struct {
	x, y, z float64
}

func (u Vec) add(v Vec) Vec         { return Vec{u.x + v.x, u.y + v.y, u.z + v.z} }
func (u Vec) sub(v Vec) Vec         { return Vec{u.x - v.x, u.y - v.y, u.z - v.z} }
func neg(v Vec) Vec                 { return Vec{-v.x, -v.y, -v.z} }
func (u Vec) dot(v Vec) float64     { return u.x*v.x + u.y*v.y + u.z*v.z }
func (u Vec) mul(a float64) Vec     { return Vec{u.x * a, u.y * a, u.z * a} }
func (u Vec) div(a float64) Vec     { return Vec{u.x / a, u.y / a, u.z / a} }
func (u Vec) elementwise(v Vec) Vec { return Vec{u.x * v.x, u.y * v.y, u.z * v.z} }
func norm(v Vec) float64            { return math.Sqrt(v.dot(v)) }
func normalize(v Vec) Vec           { return v.div(norm(v)) }
func randn() Vec                    { return Vec{rand.Float64() - 0.5, rand.Float64() - 0.5, rand.Float64() - 0.5} }

type Color = Vec

type Ray struct {
	origin, dir Vec
}

type GenericBody struct {
	center      Vec
	color       Color
	diffuseness float64
	isLamp      bool
}

func (b *GenericBody) get() *GenericBody { return b }

func (b *GenericBody) lambertFactor(lambert float64) float64 {
	return 1.0 - (1.0-lambert)*b.diffuseness
}

type Body interface {
	get() *GenericBody
	intersect(ray *Ray) (found bool, point, normal Vec)
}

type Box struct {
	GenericBody
	halfsize Vec
}

func within(x, y, xmin, ymin, xmax, ymax float64) bool {
	return (x > xmin) && (x < xmax) && (y > ymin) && (y < ymax)
}

func (b *Box) intersect(ray *Ray) (found bool, point, normal Vec) {
	if math.Abs(ray.dir.z) > 1e-9 { // xy
		side := 1.0
		if ray.dir.z > 0.0 {
			side = -1.0
		}

		if factor := (b.center.z + side*b.halfsize.z - ray.origin.z) / ray.dir.z; factor > 0.1 {
			point = ray.origin.add(ray.dir.mul(factor))
			if within(
				point.x, point.y,
				b.center.x-b.halfsize.x, b.center.y-b.halfsize.y,
				b.center.x+b.halfsize.x, b.center.y+b.halfsize.y) {
				normal = Vec{0, 0, side}
				return true, point, normal
			}
		}
	}

	if math.Abs(ray.dir.x) > 1e-9 { // yz
		side := 1.0
		if ray.dir.x > 0.0 {
			side = -1.0
		}

		if factor := (b.center.x + side*b.halfsize.x - ray.origin.x) / ray.dir.x; factor > 0.1 {
			point = ray.origin.add(ray.dir.mul(factor))
			if within(
				point.y, point.z,
				b.center.y-b.halfsize.y, b.center.z-b.halfsize.z,
				b.center.y+b.halfsize.y, b.center.z+b.halfsize.z) {
				normal = Vec{side, 0, 0}
				return true, point, normal
			}
		}
	}

	if math.Abs(ray.dir.y) > 1e-9 { // zx
		side := 1.0
		if ray.dir.y > 0.0 {
			side = -1.0
		}

		if factor := (b.center.y + side*b.halfsize.y - ray.origin.y) / ray.dir.y; factor > 0.1 {
			point = ray.origin.add(ray.dir.mul(factor))
			if within(
				point.z, point.x,
				b.center.z-b.halfsize.z, b.center.x-b.halfsize.x,
				b.center.z+b.halfsize.z, b.center.x+b.halfsize.x) {
				normal = Vec{0, side, 0}
				return true, point, normal
			}
		}
	}

	return false, point, normal
}

type Sphere struct {
	GenericBody
	radius float64
}

func (s *Sphere) intersect(ray *Ray) (found bool, point, normal Vec) {
	displacement := s.center.sub(ray.origin)
	proj := displacement.dot(ray.dir)
	discr := s.radius*s.radius + proj*proj - displacement.dot(displacement)

	if discr > 0 {
		factor := proj - math.Sqrt(discr)
		if factor > 0.1 {
			point := ray.origin.add(ray.dir.mul(factor))
			normal := (point.sub(s.center)).div(s.radius)
			return true, point, normal
		}
	}

	return false, point, normal
}

const width, height = 640, 480

type Scene struct {
	ambientColor Color
	body         []Body
}

func (sc *Scene) trace(ray *Ray, depth int) Color {
	if depth > 3 {
		return sc.ambientColor
	}

	// find nearest intersection
	bestDist := 1e9
	bestIndex := -1
	var bestPoint, bestNormal Vec

	for i, b := range sc.body {

		if found, point, normal := b.intersect(ray); found {
			dist := norm(point.sub(ray.origin))
			if dist < bestDist {
				bestDist = dist
				bestIndex = i
				bestPoint = point
				bestNormal = normal
			}
		}
	}

	// reflect rays
	if bestIndex >= 0 {
		bestBody := sc.body[bestIndex].get()

		if bestBody.isLamp {
			return bestBody.get().color
		}

		specularDir := ray.dir.sub(bestNormal.mul(2.0 * (ray.dir.dot(bestNormal))))
		diffuseDir := normalize(specularDir.add(randn().mul(2.0 * bestBody.diffuseness)))

		lambert := diffuseDir.dot(bestNormal)
		if lambert < 0 {
			diffuseDir = diffuseDir.sub(bestNormal.mul(2.0 * lambert))
			lambert = -lambert
		}

		diffuseRay := Ray{bestPoint, diffuseDir}

		return (sc.trace(&diffuseRay, depth+1).mul(bestBody.lambertFactor(lambert))).elementwise(bestBody.color)
	}

	return sc.ambientColor
}

func (sc *Scene) draw(bitmap [][width][]byte, startHeight, endHeight int,
	width, height, focal int, pos Vec, azimuth float64, antialiasing float64, done chan bool) {
	const rays = 100
	sinAz, cosAz := math.Sin(azimuth), math.Cos(azimuth)

	for i := startHeight; i < endHeight; i++ {
		for j := 0; j < width; j++ {
			dir := Vec{float64(j - width/2), float64(i - height/2), float64(focal)}

			rotDir := Vec{
				dir.x*cosAz + dir.z*sinAz,
				dir.y,
				-dir.x*sinAz + dir.z*cosAz}

			color := Color{0, 0, 0}
			for r := 0; r < rays; r++ {
				randomDir := rotDir.add(randn().mul(antialiasing))
				ray := Ray{pos, normalize(randomDir)}
				color = color.add(sc.trace(&ray, 0))
			}
			color = color.mul(255.0 / float64(rays))

			bitmap[i-startHeight][j] = []byte{byte(color.x), byte(color.y), byte(color.z)}
		}
		fmt.Printf("%3d / %3d\n", i+1, height)
	}
	done <- true
}

func main() {
	// define scene
	scene := Scene{Color{0.2, 0.2, 0.2}, nil}

	scene.body = append(
		scene.body,
		&Box{
			GenericBody{
				Vec{500, -100, 1200},
				Color{0.4, 0.7, 1.0},
				0.1,
				false},
			Vec{400, 600, 300}.div(2)})

	scene.body = append(
		scene.body,
		&Box{
			GenericBody{
				Vec{550, 210, 1100},
				Color{0.9, 1.0, 0.6},
				0.3,
				false},
			Vec{1000, 20, 1000}.div(2)})

	scene.body = append(
		scene.body,
		&Sphere{
			GenericBody{
				Vec{600, 0, 700},
				Color{1.0, 0.4, 0.6},
				0.2,
				false},
			200})

	scene.body = append(
		scene.body,
		&Sphere{
			GenericBody{
				Vec{330, 150, 700},
				Color{1.0, 1.0, 0.3},
				0.15,
				false},
			50})

	// define light
	scene.body = append(
		scene.body,
		&Sphere{
			GenericBody{
				Vec{500, -1000, -700},
				Color{1.0, 1.0, 1.0},
				1.0,
				true},
			800})

	// define eye
	pos := Vec{0, 0, 0}
	azimuth := 30.0 * math.Pi / 180.0

	const focal = 500
	const antialiasing = 1.0

	var bitmap [height][width][]byte

	// render scene
	const workers = 1
	const stripHeight = height / workers

	done := make(chan bool)

	fmt.Printf("Starting rendering with %d workers\n", workers)
	startTime := time.Now()

	for startHeight := 0; startHeight < height; startHeight += stripHeight {
		go scene.draw(bitmap[startHeight:startHeight+stripHeight], startHeight, startHeight+stripHeight,
			width, height, focal, pos, azimuth, antialiasing, done)
	}

	for w := 0; w < workers; w++ {
		<-done
	}

	endTime := time.Now()
	fmt.Printf("Rendering time = %v\n", endTime.Sub(startTime))

	// open output file
	fmt.Printf("Writing output file\n")

	f, err := os.Create("scene.ppm")
	if err != nil {
		panic(err)
	}

	fmt.Fprintf(f, "P6\n%d %d\n255\n", width, height)

	for i := 0; i < height; i++ {
		for j := 0; j < width; j++ {
			f.Write(bitmap[i][j])
		}
	}

	f.Close()
}
